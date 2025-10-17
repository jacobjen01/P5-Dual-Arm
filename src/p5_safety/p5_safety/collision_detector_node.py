import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool



class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector_node')

        self.WORLD_FRAME = "mir"

        self.ROBOT_A_CYL_FRAME_DATA = [("alice_upper_arm_link_shifted", "alice_forearm_link_shifted", 0.05), ("alice_forearm_link", "alice_wrist_1_link_shifted", 0.05),
                                                           ("alice_wrist_1_link", "alice_wrist_2_link", 0.01), ("alice_wrist_2_link", "alice_wrist_3_link", 0.01)]
        self.ROBOT_B_CYL_FRAME_DATA = [("bob_upper_arm_link_shifted", "bob_forearm_link_shifted", 0.05), ("bob_forearm_link", "bob_wrist_1_link_shifted", 0.05),
                                                           ("bob_wrist_1_link", "bob_wrist_2_link", 0.01), ("bob_wrist_2_link", "bob_wrist_3_link", 0.01)]

        self.SAFETY_MARGIN = 0.1 # Safety margin in meters. the minimum distance allowed between the two robots.

        self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS sat to focus on prioritizing the latest signal
        protective_stop_publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.protective_stop_publisher = self.create_publisher(Bool, "protective_stop", protective_stop_publisher_qos)

        self.timer_create_missing_tf_trees = self.create_timer(0.1, self.create_missing_joint_tf_trees)
        self.timer_check_collision = self.create_timer(0.1, self.detect_collision)


    """
    Helper function to quickly create tf trees
    """
    def _create_tf_tree(self, parent_name, child_name, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_name
        t.child_frame_id = child_name

        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]

        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]

        self.tf_broadcaster.sendTransform(t)


    """
    Create the transformation trees which can be used to create a vector from joint 1 to joint 2
    """
    def create_missing_joint_tf_trees(self):
        self._create_tf_tree("alice_upper_arm_link", "alice_upper_arm_link_shifted", [0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 1.0])
        self._create_tf_tree("alice_forearm_link", "alice_forearm_link_shifted", [0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 1.0])
        self._create_tf_tree("alice_wrist_1_link", "alice_wrist_1_link_shifted", [0.0, 0.0, -0.13, 0.0, 0.0, 0.0, 1.0])
        self._create_tf_tree("bob_upper_arm_link", "bob_upper_arm_link_shifted", [0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 1.0])
        self._create_tf_tree("bob_forearm_link", "bob_forearm_link_shifted", [0.0, 0.0, 0.13, 0.0, 0.0, 0.0, 1.0])
        self._create_tf_tree("bob_wrist_1_link", "bob_wrist_1_link_shifted", [0.0, 0.0, -0.13, 0.0, 0.0, 0.0, 1.0])


    """
    Get the coordinate for one transformation tree in respect to another
    """
    def _get_tf_tree_crd(self, parent_name, child_name):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(parent_name, child_name, now)

            crd = np.array([trans.transform.translation.x, trans.transform.translation.y,
                            trans.transform.translation.z])

            return crd

        except Exception as e:
            self.get_logger().error(f'Failed to get transform from {parent_name}/{child_name}: {e}')
            self.error_handler.report_error(self.error_handler.info, f'Failed to get transform from {parent_name}/{child_name}: {e}')

            return []


    """
    Get the position of all line points for one robot.
    """
    def _get_robot_cylinder_desc(self, robot_identifier):
        cylinder_desc = []

        for parent_name, child_name, radius in robot_identifier:

            pos_parent = self._get_tf_tree_crd(self.WORLD_FRAME, parent_name)
            pos_child = self._get_tf_tree_crd(self.WORLD_FRAME, child_name)

            if len(pos_parent) > 0 and len(pos_child) > 0:
                 cylinder_desc.append([pos_parent, pos_child, radius])
            else:
                return []

        return cylinder_desc


    """
    Help function to see if in length
    """
    def _check_length(self, dist, radius1, radius2):
        if dist < radius1 + radius2 + self.SAFETY_MARGIN:
            return True
        return False


    """
    Help function to check threshold
    """
    def _check_threshold(self, point, outer_points):
        if outer_points[0][0] < point[0] < outer_points[1][0] or outer_points[1][0] < point[0] < outer_points[0][0]:
            if outer_points[0][1] < point[1] < outer_points[1][1] or outer_points[1][1] < point[1] < outer_points[0][1]:
                if outer_points[0][2] < point[2] < outer_points[1][2] or outer_points[1][2] < point[2] < outer_points[0][2]:
                    return True

        return False


    """
    Detecting if two cylinders collides
    """
    def _cylinder_detect_collision(self, cylinder_desc_1, cylinder_desc_2):
        cyl_1_crds = cylinder_desc_1[0:2]
        cyl_1_radius = cylinder_desc_1[2]
        cyl_1_vec = cyl_1_crds[0] - cyl_1_crds[1]
        cyl_1_begin_crd = cyl_1_crds[0]

        cyl_2_crds = cylinder_desc_2[0:2]
        cyl_2_radius = cylinder_desc_2[2]
        cyl_2_vec = cyl_2_crds[0] - cyl_2_crds[1]
        cyl_2_begin_crd = cyl_2_crds[0]

        # Lines are parallel
        if np.linalg.norm(np.cross(cyl_1_vec, cyl_2_vec)) == 0:
            dist = np.linalg.norm(np.cross((cyl_1_begin_crd - cyl_2_begin_crd), cyl_1_vec)) / np.linalg.norm(cyl_1_vec)
            return self._check_length(dist, cyl_1_radius, cyl_2_radius)

        # Lines are not parallel, calculating distance
        dist = (np.abs(np.dot((cyl_2_begin_crd - cyl_1_begin_crd), np.cross(cyl_1_vec, cyl_2_vec))) /
                np.linalg.norm(np.cross(cyl_1_vec, cyl_2_vec)))

        # If the closest distance is greater than the minimum
        if not self._check_length(dist, cyl_1_radius, cyl_2_radius):
            return False

        # If not, the points are calculated for closest distance
        n = np.cross(cyl_1_vec, cyl_2_vec)

        t1 = np.dot(np.cross(cyl_2_vec, n), (cyl_2_begin_crd - cyl_1_begin_crd)) / np.dot(n, n)
        t2 = np.dot(np.cross(cyl_1_vec, n), (cyl_2_begin_crd - cyl_1_begin_crd)) / np.dot(n, n)

        closest_point_1 = t1*cyl_1_vec + cyl_1_begin_crd
        closest_point_2 = t2*cyl_2_vec + cyl_2_begin_crd

        # If both points are inside
        if self._check_threshold(closest_point_1, cyl_1_crds) and self._check_threshold(closest_point_2, cyl_2_crds):
            return True

        point_distances = []
        point_to_vec_distances = []

        for p1 in cyl_1_crds:
            for p2 in cyl_2_crds:
                point_distances.append(np.linalg.norm(p1 - p2))

        if self._check_length(min(point_distances), cyl_1_radius, cyl_2_radius):
            return True

        # Gets the distances from point to line and checks if they are in threshold.
        for p1 in cyl_1_crds:
            vec = p1 - cyl_2_begin_crd
            proj = np.dot(vec, cyl_2_vec) / np.linalg.norm(cyl_2_vec)**2 * cyl_2_vec

            if self._check_threshold(proj + cyl_2_begin_crd, cyl_2_crds):
                point_to_vec_distances.append(np.linalg.norm(np.cross((cyl_2_begin_crd - p1), cyl_2_vec)) / np.linalg.norm(cyl_2_vec))

        for p2 in cyl_2_crds:
            vec = p2 - cyl_1_begin_crd
            proj = np.dot(vec, cyl_1_vec) / np.linalg.norm(cyl_1_vec)**2 * cyl_1_vec

            if self._check_threshold(proj + cyl_1_begin_crd, cyl_1_crds):
                point_to_vec_distances.append(np.linalg.norm(np.cross(cyl_1_begin_crd - p2, cyl_1_vec)) / np.linalg.norm(cyl_1_vec))

        # Checks distance
        if len(point_to_vec_distances) > 0:
            if self._check_length(min(point_to_vec_distances), cyl_1_radius, cyl_2_radius):
                return True

        return False


    """
    Check whether the system detects any collision. 
    """
    def detect_collision(self):
        try:
            cylinder_desc_robot_1 = self._get_robot_cylinder_desc(self.ROBOT_A_CYL_FRAME_DATA)
            cylinder_desc_robot_2 = self._get_robot_cylinder_desc(self.ROBOT_B_CYL_FRAME_DATA)

            if len(cylinder_desc_robot_1) == 0 or len(cylinder_desc_robot_2) == 0:
                return

            msg = Bool()

            for cylinder_desc_1 in cylinder_desc_robot_1:
                for cylinder_desc_2 in cylinder_desc_robot_2:

                    if self._cylinder_detect_collision(cylinder_desc_1, cylinder_desc_2):
                        msg.data = True
                        self.protective_stop_publisher.publish(msg)

                        return
                    else:
                        continue

            msg.data = False
            self.protective_stop_publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Fatal error: {e}')
            self.error_handler.report_error(self.error_handler.fatal, f'{e}')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
