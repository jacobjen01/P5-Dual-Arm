import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from std_msgs.msg import String
from p5_interfaces.srv import MoveToPose, SetLinearMovement, SetReferenceFrame
from p5_interfaces.msg import Tagvector

class RelativeMover(Node):
    def __init__(self):
        super().__init__('relative_mover_node')

        self.MAX_VELOCITY = 0.2 # 200 mm/s
        self.ACCELERATION = 0.1 # 500 mm/s^2
        self.INTERP_ITERATIONS = 10 # number of times the point estimator should update.
        self.UPDATE_RATE = 100 # Number of times the system shall update per second

        self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('robot_prefix', 'alice')
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value

        self.linear_movement = False
        self.linear_movement_use_tracking_velocity = False
        self.reference_frame = None

        self.timer_create_goal_frame = None
        self.timer_get_goal_pose_respect_to_base = None

        self.goal_pose_rel_target_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.goal_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame_theoretical = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame_start_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        self.goal_pose_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_theoretical_velocity = 0.0
        self.previous_robot_poses = []

        self.i = 0

        self.pose_publisher = self.create_publisher(PoseStamped, f"{self.robot_prefix}/servo_node/pose_target_cmds", 10) #f"{self.robot_prefix}_robot_pose_for_admittance_control"
        self.velocity_subscriber = self.create_subscription(Tagvector, "/future_tag_vector", self.get_goal_velocity_callback, 10)

        # self.set_linear_movement_service = self.create_service(SetLinearMovement, 'set_linear_movement', self.set_linear_movement_callback)
        # self.set_tf_tree_service = self.create_service(SetReferenceFrame, 'set_reference_frame', self.set_reference_frame_callback)
        self.move_to_pose_service = self.create_service(MoveToPose, f'{self.robot_prefix}/move_to_pose', self.move_to_pose_callback)

        self.timer_get_ee_pose_respect_to_base = self.create_timer(1 / self.UPDATE_RATE,
                                                                   self.get_ee_pose_respect_to_base)

    # """
    # Function callback from service to determine if the robot shall move in cartesian or joint space.
    # """
    # def set_linear_movement_callback(self, request, response):
    #     try:
    #         self.linear_movement = request.linear
    #         self.linear_movement_use_tracking_velocity = request.use_tracking_velocity
    #
    #         self.get_logger().info(f"Received request for linear movement {self.linear_movement}, {self.linear_movement_use_tracking_velocity}")
    #
    #         response.resp = True
    #         return response
    #
    #     except Exception as e:
    #         self.get_logger().error(e)
    #         self.error_handler.report_error(self.error_handler.fatal, f"Failed to send true response. Error: {e}")
    #
    #         response.resp = False
    #         return response
    #
    # """
    # Function to set the reference frame, the robot shall move in respect to.
    # """
    # def set_reference_frame_callback(self, request, response):
    #     self.reference_frame = None
    #     try:
    #         self.reference_frame = request.frame
    #
    #         self.get_logger().info(f"Received request for reference frame {self.reference_frame}")
    #
    #         response.resp = True
    #         return response
    #
    #     except Exception as e:
    #         self.get_logger().error(e)
    #         self.error_handler.report_error(self.error_handler.fatal, f"Failed to send true response. Error: {e}")
    #
    #         response.resp = False
    #         return response

    """
    Function callback to execute linear movement.
    """
    def move_to_pose_callback(self, request, response):
        try:
            self.linear_movement = request.linear
            self.linear_movement_use_tracking_velocity = request.use_tracking_velocity
            self.reference_frame = request.frame

            pos = request.pose.position
            quat = request.pose.orientation

            self.goal_pose_rel_target_frame = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
            self.ee_pose_rel_base_frame_start_frame = self.ee_pose_rel_base_frame.copy()
            self.ee_pose_rel_base_frame_theoretical = self.ee_pose_rel_base_frame.copy()

            self.get_logger().info(f"Received request for pose {self.goal_pose_rel_target_frame}")

            self.timer_create_goal_frame = self.create_timer(1 / self.UPDATE_RATE, self.create_current_goal_frame)
            self.timer_get_goal_pose_respect_to_base = self.create_timer(1 / self.UPDATE_RATE,
                                                                         self.get_goal_pose_respect_to_base)

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

    def get_goal_velocity_callback(self, msg):
        for vector in msg.vectors:
            if vector.tag_id == self.reference_frame:
                self.goal_pose_velocity = np.array([vector.vx_unit, vector.vy_unit, vector.vz_unit])
                self.get_logger().info(f"Received velocity {[vector.vx_unit, vector.vy_unit, vector.vz_unit]}")


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
            self.get_logger().error(f'Failed to create transform from {self.reference_frame}"/{self.robot_prefix}_goal_frame: {e}')
            self.error_handler.report_error(self.error_handler.info,
                                            f'Failed to create transform from {self.reference_frame}"/{self.robot_prefix}_goal_frame: {e}')

    """
    Helper function goal pose respect to base frame.
    """
    def get_goal_pose_respect_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link", f"{self.robot_prefix}_goal_frame",
                                                    now)

            crd = trans.transform.translation
            quat = trans.transform.rotation

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
            t = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link", f"{self.robot_prefix}_tool0", now)

            crd = t.transform.translation
            quat = t.transform.rotation


            self.ee_pose_rel_base_frame = [crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w]

            self.previous_robot_poses.append(np.array(self.ee_pose_rel_base_frame.copy())[0:3])

            if len(self.previous_robot_poses) >= self.UPDATE_RATE * 0.5:
                self.previous_robot_poses.pop(0)

            vec = self.previous_robot_poses[-1] - self.previous_robot_poses[0]

            self.robot_velocity = vec[0:3] / 0.5

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
    Helper function to get estimated goal pose in respect to base frame of robot, for robot to move to
    """
    def _get_estimated_goal_pose(self):
        vec = np.array(self.goal_pose_rel_base_frame[0:3]) - np.array(self.ee_pose_rel_base_frame[0:3])
        quat = np.array(self.goal_pose_rel_base_frame[3:7])

        t = self._get_movement_time_to_goal(vec, self.robot_velocity)

        vec_org = vec.copy()

        for i in range(self.INTERP_ITERATIONS):
            vec = vec_org + t * self.goal_pose_velocity
            t = self._get_movement_time_to_goal(vec, self.robot_velocity)

        crd = vec + np.array(self.ee_pose_rel_base_frame[0:3])

        return np.concatenate([crd, quat])

    """
    Helper function for calculating estimated robot movement time
    """
    def _get_movement_time_to_goal(self, vec, vel):
        vel_proj = np.dot(vec, vel) / np.dot(vec, vec) * vec
        v0 = np.linalg.norm(vel_proj) * (2 * np.heaviside(np.dot(vec, vel), 1) - 1) # Gets speed for robot

        v0 = self.robot_theoretical_velocity

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
        # t = Pose()
        #
        # t.position.x = pose[0]
        # t.position.y = pose[1]
        # t.position.z = pose[2]
        # t.orientation.x = pose[3]
        # t.orientation.y = pose[4]
        # t.orientation.z = pose[5]
        # t.orientation.w = pose[6]
        #
        # self.pose_publisher.publish(t)

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "alice_base_link"  # "link_base"

        pose_goal.pose.position.x = pose[0]
        pose_goal.pose.position.y = pose[1]
        pose_goal.pose.position.z = pose[2]
        pose_goal.pose.orientation.x = pose[3]
        pose_goal.pose.orientation.y = pose[4]
        pose_goal.pose.orientation.z = pose[5]
        pose_goal.pose.orientation.w = pose[6]

        pose_goal.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher.publish(pose_goal)

    """
    Linear motion_predictor
    """
    def _linear_motion_predictor(self, goal_pose_rel_base_frame):
        crd_ee_start = np.array(self.ee_pose_rel_base_frame_start_frame[0:3])
        crd_ee = np.array(self.ee_pose_rel_base_frame_theoretical[0:3]) #np.array(self.ee_pose_rel_base_frame[0:3])
        crd_goal = np.array(goal_pose_rel_base_frame[0:3])
        quat_ee = np.array(self.ee_pose_rel_base_frame[3:7])
        quat_goal = np.array(goal_pose_rel_base_frame[3:7])

        # self.get_logger().info(f"crd_ee_start: {crd_ee_start}, crd_ee: {crd_ee}, crd_goal: {crd_goal}, quat_ee_start: {quat_ee_start}, quat_goal: {quat_goal}")

        dist = np.linalg.norm(crd_goal - crd_ee)

        vel = self.robot_theoretical_velocity #np.linalg.norm(self.robot_velocity) #

        if dist < 0.01:
            return np.concatenate([crd_goal, quat_goal])

        if dist <= 1/2 * vel**2 / self.ACCELERATION:
            a = vel**2 / (2*dist) * -1
            v = vel

        elif vel >= self.MAX_VELOCITY:
            a = 0
            v = self.MAX_VELOCITY

        else:
            v = vel
            a = self.ACCELERATION

        step = v * 1 / self.UPDATE_RATE + 1 / 2 * a * (1 / self.UPDATE_RATE) ** 2
        frac = 1 - dist / np.linalg.norm(crd_goal-crd_ee_start)

        frac = float(np.clip(frac, 0.0, 1.0))

        vec = (crd_ee + (crd_goal - crd_ee) / dist * step) - crd_ee_start
        vec_proj_onto = crd_goal - crd_ee_start
        proj = np.dot(vec, vec_proj_onto) / np.linalg.norm(vec_proj_onto) ** 2 * vec_proj_onto

        new_crd = proj + crd_ee_start

        self.robot_theoretical_velocity += a * (1/self.UPDATE_RATE)

        quat_ee /= np.linalg.norm(quat_ee) + 1e-16
        quat_goal /= np.linalg.norm(quat_goal) + 1e-16

        if np.dot(quat_ee, quat_goal) < 0.0:
            quat_goal = -quat_goal

        r_pair = R.from_quat(np.vstack([quat_ee, quat_goal]))
        slerp = Slerp([0.0, 1.0], r_pair)
        new_quat = slerp([frac]).as_quat()[0]

        new_pose = np.concatenate([new_crd, new_quat])

        self.ee_pose_rel_base_frame_theoretical = new_pose

        # if self.i % 100 == 0:
        #     self.get_logger().info(f"start crd {crd_ee}, goal crd {crd_goal}, new crd {new_crd}")
        #     self.get_logger().info(f"step dir {(crd_goal - crd_ee) / dist}, step {step} ")
        #     self.get_logger().info(f"frac: {frac}, at 0 {slerp([0.0]).as_quat()[0]}, at 1 {slerp([1.0]).as_quat()[0]}")
        #
        #     self.get_logger().info(f"goal pose: {goal_pose_rel_base_frame}")
        #     self.get_logger().info(f"new pose: {new_pose}")
        #
        #     t = TransformStamped()
        #     t.header.stamp = self.get_clock().now().to_msg()
        #     t.header.frame_id = self.reference_frame
        #     t.child_frame_id = f"{self.robot_prefix}_test_frame"
        #
        #     t.transform.translation.x = new_pose[0]
        #     t.transform.translation.y = new_pose[1]
        #     t.transform.translation.z = new_pose[2]
        #
        #     t.transform.rotation.x = new_pose[3]
        #     t.transform.rotation.y = new_pose[4]
        #     t.transform.rotation.z = new_pose[5]
        #     t.transform.rotation.w = new_pose[6]
        #
        #     self.tf_broadcaster.sendTransform(t)
        #
        # self.i += 1

        return new_pose


def main(args=None):
    rclpy.init(args=args)
    node = RelativeMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
