import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from geometry_msgs.msg import Pose, String, TransformStamped
from p5_interfaces.srv import MoveToPose, SetLinearMovement, SetReferenceFrame


class RelativeMover(Node):
    def __init__(self):
        super().__init__('relative_mover_node')

        self.MAX_VELOCITY = 0.2 # 200 mm/s
        self.ACCELERATION = 0.5 # 500 mm/s^2
        self.INTERP_ITERATIONS = 10 # number of times the point estimator should update.
        self.UPDATE_RATE = 20 # Number of times the system shall update per second

        self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('robot_prefix', 'alice')
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value

        self.linear_movement = False
        self.linear_movement_use_tracking_velocity = False
        self.reference_frame = None

        self.goal_pose_rel_target_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.goal_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame_start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        self.goal_pose_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_velocity = np.array([0.0, 0.0, 0.0])

        self.pose_publisher = self.create_publisher(Pose, f"{self.robot_prefix}_robot_pose_for_admittance_control", 10)

        self.set_linear_movement_service = self.create_service(SetLinearMovement, 'set_linear_movement', self.set_linear_movement_callback)
        self.move_to_pose_service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)
        self.set_tf_tree_service = self.create_service(SetReferenceFrame, 'set_reference_frame', self.set_reference_frame_callback)

        self.timer_create_goal_frame = self.create_timer(1 / self.UPDATE_RATE, self.create_current_goal_frame)
        self.timer_get_goal_pose_respect_to_base = self.create_timer(1 / self.UPDATE_RATE, self.get_goal_pose_respect_to_base)
        self.timer_get_ee_pose_respect_to_base = self.create_timer(1 / self.UPDATE_RATE, self.get_ee_pose_respect_to_base)

    """
    Function callback from service to determine if the robot shall move in cartesian or joint space.  
    """
    def set_linear_movement_callback(self, request, response):
        try:
            self.linear_movement = request.linear
            self.linear_movement_use_tracking_velocity = request.use_tracking_velocity

            response.resp = True
            return response

        except Exception as e:
            self.getlogger().error(e)
            self.error_handler.report_error(self.error_handler.fatal, f"Failed to send true response. Error: {e}")

            response.resp = False
            return response

    """
    Function to set the reference frame, the robot shall move in respect to.
    """
    def set_reference_frame_callback(self, request, response):
        self.reference_frame = None
        try:
            self.reference_frame = request.frame

            response.resp = True
            return response

        except Exception as e:
            self.getlogger().error(e)
            self.error_handler.report_error(self.error_handler.fatal, f"Failed to send true response. Error: {e}")

            response.resp = False
            return response

    """
    Function callback to execute linear movement.
    """
    def move_to_pose_callback(self, request, response):
        try:
            pos = request.transform.translation
            quat = request.transform.orientation

            self.goal_pose_rel_target_frame = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
            self.ee_pose_rel_base_frame_start_pose = self.ee_pose_rel_base_frame.copy()

            self.timer_move_robot = self.create_timer(1/self.UPDATE_RATE, self.move_to_pose)

            response.resp = True
            return response

        except Exception as e:
            self.get_logger().error(f'Failed to execute relative movement. Error {e}')
            self.error_handler.report_error(self.error_handler.fatal,
                                            f'Failed to execute relative movement. Error {e}')
            response.resp = False
            return response

    """
    Callback functions for topics containing value about robot velocity and goal velocity.
    """
    def get_robot_velocity_callback(self):
        pass

    def get_goal_velocity_callback(self):
        pass

    """
    Creates the goal frame which the robot should move to
    """
    def create_current_goal_frame(self):
        if self.reference_frame is None:
            return

        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.reference_frame
            t.child_frame_id = f"{self.robot_prefix}_goal_frame"

            t.transform.translation.x = self.goal_pose_rel_target_frame[0]
            t.transform.translation.y = self.goal_pose_rel_target_frame[1]
            t.transform.translation.z = self.goal_pose_rel_target_frame[2]

            t.transform.rotation.x = self.goal_pose_rel_target_frame[3]
            t.transform.rotation.y = self.goal_pose_rel_target_frame[4]
            t.transform.rotation.z = self.goal_pose_rel_target_frame[5]
            t.transform.rotation.w = self.goal_pose_rel_target_frame[6]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Failed to get transform from {self.reference_frame}"/{self.robot_prefix}_goal_frame: {e}')
            self.error_handler.report_error(self.error_handler.info,
                                            f'Failed to get transform from {self.reference_frame}"/{self.robot_prefix}_goal_frame: {e}')

    """
    Helper function goal pose respect to base frame.
    """
    def get_goal_pose_respect_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link", f"{self.robot_prefix}_goal_frame",
                                                    now)

            crd = trans.transform.translation
            quat = trans.orientation.orientation

            self.goal_pose_rel_base_frame = [crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w]

        except Exception as e:
            self.get_logger().error(
                f'Failed to get transform from {self.robot_prefix}_base_link"/{self.robot_prefix}_goal_frame: {e}')
            self.error_handler.report_error(self.error_handler.info,
                                            f'Failed to get transform from {self.robot_prefix}_base_link"/{self.robot_prefix}_goal_frame: {e}')

    """
    Helper function to get end-effector frame respect to base frame.
    """
    def get_ee_pose_respect_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link", f"{self.robot_prefix}_tool0", now)

            crd = trans.transform.translation
            quat = trans.orientation.orientation

            self.ee_pose_rel_base_frame = [crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w]

        except Exception as e:
            self.get_logger().error(
                f'Failed to get transform from {self.robot_prefix}_base_link"/{self.robot_prefix}_goal_frame: {e}')
            self.error_handler.report_error(self.error_handler.info,
                                            f'Failed to get transform from {self.robot_prefix}_base_link"/{self.robot_prefix}_goal_frame: {e}')

    """
    Continuously moves the robot towards frame.
    """
    def move_to_pose(self):
        try:
            if not self.linear_movement:
                self._publish_pose(self.goal_pose_rel_base_frame)
                return

            if self.linear_movement_use_tracking_velocity:
                est_pose = self._get_estimated_goal_pose()
                next_pose = self._linear_motion_predictor(est_pose)

                self._publish_pose(next_pose)
                return

            next_pose = self._linear_motion_predictor(self.goal_pose_rel_base_frame)
            self._publish_pose(next_pose)


        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.error_handler.report_error(self.error_handler.error,
                                            f'Error: {e}')

    """
    Helper function to get estimated goal pose in respect to base frame of robot, for robot to move to.
    """
    def _get_estimated_goal_pose(self):
        vec = np.array(self.goal_pose_rel_base_frame[0:3]) - np.array(self.ee_pose_rel_base_frame[0:3])
        quat = np.array(self.goal_pose_rel_base_frame[3:7])

        t = self._get_movement_time_to_goal(vec, self.robot_velocity)

        for i in range(self.INTERP_ITERATIONS):
            vec = vec + t * self.goal_pose_velocity
            t = self._get_movement_time_to_goal(vec, self.robot_velocity)

        crd = vec + np.array(self.ee_pose_rel_base_frame[0:3])

        return np.concatenate([crd, quat])

    """
    Helper function for calculating estimated robot movement time
    """
    def _get_movement_time_to_goal(self, vec, vel):
        vel_proj = np.dot(vec, vel) / np.dot(vec, vec) * vec
        v0 = np.linalg.norm(vel_proj) * (2 * np.heaviside(np.dot(vec, vel), 1) - 1) # Gets speed for robot

        dist = np.linalg.norm(vec)

        t1 = (self.MAX_VELOCITY - v0) / self.ACCELERATION
        d1 = (self.MAX_VELOCITY + v0) / 2 * t1

        t2 = self.MAX_VELOCITY / self.ACCELERATION
        d2 = self.MAX_VELOCITY / 2 * t2

        if d1+d2 < dist: #Trapetz velocity profile
            dc = dist - (d1 + d2)
            tc = dc / self.MAX_VELOCITY

            return t1+t2+tc

        vp = (np.sqrt(2) * np.sqrt(v0**2 + 2 * dist * self.ACCELERATION)) / 2

        t1 = (vp - v0) / self.ACCELERATION
        t2 = vp / self.ACCELERATION

        return t1+t2

    """
    Publishes goal pose
    """
    def _publish_pose(self, pose):
        t = Pose()

        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]

        self.pose_publisher.publish(t)

    """
    Linear motion_predictor
    """
    def _linear_motion_predictor(self, pose_rel_to_base):
        crd_ee_start = np.array(self.ee_pose_rel_base_frame_start_pose[0:3])
        crd_ee = np.array(self.ee_pose_rel_base_frame[0:3])
        crd_goal = np.array(pose_rel_to_base[0:3])
        quat_ee_start = np.array(self.ee_pose_rel_base_frame_start_pose[3:7])
        quat_goal = np.array(pose_rel_to_base[3:7])

        dist = np.linalg.norm(crd_goal - crd_ee)
        vel = np.linalg.norm(self.robot_velocity)

        percentage_moved = dist / np.linalg.norm(crd_ee_start - crd_goal)

        if dist <= 1/2 * self.MAX_VELOCITY**2 / self.ACCELERATION:
            de_acceleration = vel**2 / (2*dist)

            forward_factor = vel * 1/self.UPDATE_RATE - de_acceleration * (1/self.UPDATE_RATE)**2

        elif vel >= self.MAX_VELOCITY:
            forward_factor = self.MAX_VELOCITY * 1/self.UPDATE_RATE

        else:
            forward_factor = vel * 1/self.UPDATE_RATE + self.ACCELERATION * (1/self.UPDATE_RATE)**2

        new_crd = crd_ee + (crd_goal - crd_ee) * forward_factor / dist

        r_base_ee_start = R.from_quat([quat_ee_start])
        r_base_goal = R.from_quat([quat_goal])

        r_ee_goal = np.linalg.norm(r_base_ee_start.as_matrix()) @ r_base_goal.as_matrix()
        euler_ee_goal = R.from_matrix(r_ee_goal).as_euler('xyz')

        euler_ee_cp = euler_ee_goal * percentage_moved

        r_ee_cp = R.from_euler('xyz', euler_ee_cp)
        r_base_cp = r_ee_cp.as_matrix() @ r_base_goal.as_matrix()

        new_quat = R.from_matrix(r_base_cp).as_quat()

        new_pose = np.concatenate([new_crd, new_quat])
        return new_pose


def main(args=None):
    rclpy.init(args=args)
    node = RelativeMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
