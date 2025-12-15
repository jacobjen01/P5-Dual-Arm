import rclpy
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R, Slerp

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from geometry_msgs.msg import TransformStamped, PoseStamped
from p5_interfaces.srv import MoveToPose, GetStatus, StopRelativeMover
from p5_interfaces.msg import TagvectorArray, CommandState


class RelativeMover(Node):
    def __init__(self):
        super().__init__('p5_relative_mover_node')

        self.MAX_VELOCITY = 0.2  # 200 mm/s
        self.ACCELERATION = 0.05  # 500 mm/s^2
        self.INTERP_ITERATIONS = 10  # number of times the point estimator should update.
        self.UPDATE_RATE = 100  # Number of times the system shall update per second
        self.CRD_OFFSET = 0.006 # 2 mm off.
        self.ANGLE_OFFSET = 0.01 # in radians.
        self.DECIMAL_PRECISION = 2
        self.ROBOT_BASE_ORIENTATIONS = {"alice": np.array([-0.3557, -0.1534, 0.8516, 0.3531]),
                                "bob": np.array([-0.13392, 0.35155, -0.34523, 0.85983])}

        self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('robot_prefix', 'alice')
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value

        robot_R = R.from_quat(self.ROBOT_BASE_ORIENTATIONS[self.robot_prefix])
        self.velocity_rotation_matrix = np.linalg.inv(robot_R.as_matrix())
        
        self.linear_movement = False
        self.linear_movement_use_tracking_velocity = False
        self.reference_frame = None

        self.timer_create_goal_frame = None
        self.timer_get_goal_pose_respect_to_base = None

        self.reached_frame = False

        self.timer_move_robot = None

        self.goal_pose_rel_target_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.goal_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame_buffer =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]  
        self.ee_pose_rel_base_frame_theoretical = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.ee_pose_rel_base_frame_start_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        self.goal_pose_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_velocity = np.array([0.0, 0.0, 0.0])
        self.robot_theoretical_velocity = np.array([0.0, 0.0, 0.0])
        self.previous_robot_poses = []

        self.i = 0

        self.integral = np.array([0.0, 0.0, 0.0])
        self.integral_double = np.array([0.0, 0.0, 0.0])
        self.previous_error = np.array([0.0, 0.0, 0.0])

        self.pose_publisher = self.create_publisher(PoseStamped,
                                                    'p5_robot_pose_to_admittance', 10)

        self.velocity_subscriber = self.create_subscription(TagvectorArray,
                                                            "/future_tag_vector",
                                                            self.get_goal_velocity_callback, 10)

        self.move_to_pose_service = self.create_service(MoveToPose,
                                                        'p5_move_to_pose',
                                                        self.move_to_pose_callback)

        self.move_to_pose_status_service = self.create_service(GetStatus,
                                                        'p5_relative_mover_status',
                                                        self.move_to_pose_status_callback)

        self.stop_service = self.create_service(StopRelativeMover,
                                                        'p5_relative_mover_stop',
                                                        self.stop_callback)

        self.timer_get_ee_pose_respect_to_base = self.create_timer(1 / self.UPDATE_RATE,
                                                                   self.get_ee_pose_respect_to_base)

    """
    Function callback to execute linear movement.
    """
    def move_to_pose_callback(self, request, response):
        try:
            self.linear_movement = request.linear
            self.linear_movement_use_tracking_velocity = request.use_tracking_velocity

            if request.frame != self.reference_frame:
                self.integral = np.array([0.0, 0.0, 0.0])
                self.integral_double = np.array([0.0, 0.0, 0.0])
                self.previous_error = np.array([0.0, 0.0, 0.0])
            
            self.reference_frame = request.frame

            pos = request.pose.position
            quat = request.pose.orientation

            self.goal_pose_rel_target_frame = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]
            self.ee_pose_rel_base_frame_start_frame = self.ee_pose_rel_base_frame.copy()
            self.ee_pose_rel_base_frame_theoretical = self.ee_pose_rel_base_frame.copy()

            self.goal_pose_velocity = np.array([0.0, 0.0, 0.0])
            
            if self.timer_move_robot is None:
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
    Callback function to get the status of the movement.
    """
    def move_to_pose_status_callback(self, request, response):
        try:
            now = rclpy.time.Time()
            t_ee = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link", f"{self.robot_prefix}_tool0", now)
            t_goal = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link",
                                                    f"{self.robot_prefix}_goal_frame",
                                                    now)
            crd = t_ee.transform.translation
            quat = t_ee.transform.rotation

            robot_pose = np.array([crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w])
            
            crd = t_goal.transform.translation
            quat = t_goal.transform.rotation
            
            goal_pose = np.array([crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w])  

            robot_R = R.from_quat(robot_pose[3:7])
            goal_R = R.from_quat(goal_pose[3:7])

            crd_diff = np.linalg.norm(robot_pose[0:3] - goal_pose[0:3])
            r_rel = robot_R.inv() * goal_R
            angle_diff = r_rel.magnitude()

            response.running = False

            if crd_diff < self.CRD_OFFSET and angle_diff < self.ANGLE_OFFSET:
                response.running = True

            return response

        except Exception as e:
            self.get_logger().error(f'Error {e}')
            self.error_handler.report_error(self.error_handler.fatal,
                                            f'Error {e}')
            response.running = False
            return response

    """
    Stop relative mover callback
    """

    def stop_callback(self, request, response):
        try:
            response.resp = False
            if request.stop:
                self.timer_move_robot.cancel()
                self.timer_create_goal_frame.cancel()
                self.timer_get_goal_pose_respect_to_base.cancel()

                self.timer_move_robot.destroy()
                self.timer_create_goal_frame.destroy()
                self.timer_get_goal_pose_respect_to_base.destroy()

                self.timer_move_robot = None
                self.timer_create_goal_frame = None
                self.timer_get_goal_pose_respect_to_base = None

                response.resp = True

            return response

        except Exception as e:
            self.get_logger().error(f'Error {e}')
            self.error_handler.report_error(self.error_handler.fatal,
                                            f'Error {e}')
            response.resp = False
            return response

    """
    Callback functions for topics containing value about robot velocity and goal velocity.
    """
    def get_goal_velocity_callback(self, msg):
        for vector in msg.vectors:
            if vector.tag_id == self.reference_frame:
                
                vec = np.array([[vector.vx],
                                [vector.vy],
                                [vector.vz]])
                
                self.goal_pose_velocity = (self.velocity_rotation_matrix @ vec).flatten()

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
                                            f'Failed to create transform from{self.reference_frame}"/{self.robot_prefix}_goal_frame: {e}')

    """
    Helper function goal pose respect to base frame.
    """
    def get_goal_pose_respect_to_base(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f"{self.robot_prefix}_base_link",
                                                    f"{self.robot_prefix}_goal_frame",
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
        #try:
        if not self.linear_movement:
            self._publish_pose(self.goal_pose_rel_base_frame)
            return

        if self.linear_movement_use_tracking_velocity:
            # est_pose = self._get_estimated_goal_pose()
            next_pose = self._test_of_PID_controller()

            self._publish_pose(next_pose)
            return

        next_pose = self._linear_motion_predictor(self.goal_pose_rel_base_frame)
        self._publish_pose(next_pose)

        #except Exception as e:
        #    self.get_logger().error(f'Error: {e}')
        #    self.error_handler.report_error(self.error_handler.error,
        #                                    f'Error: {e}')

    """
    Helper function to get estimated goal pose in respect to base frame of robot, for robot to move to
    """
    def _get_estimated_goal_pose(self):
        vec = np.array(self.goal_pose_rel_base_frame[0:3]) - np.array(self.ee_pose_rel_base_frame[0:3])
        quat = np.array(self.goal_pose_rel_base_frame[3:7])

        t = self._get_movement_time_to_goal(vec, self.robot_theoretical_velocity)

        vec_org = vec.copy()

        for i in range(self.INTERP_ITERATIONS):
            vec = vec_org + t * self.goal_pose_velocity
            t = self._get_movement_time_to_goal(vec, self.robot_theoretical_velocity)

        crd = vec + np.array(self.ee_pose_rel_base_frame[0:3])

        return np.concatenate([crd, quat])

    """
    Helper function for calculating estimated robot movement time
    """
    def _get_movement_time_to_goal(self, vec, vel):
        vel_proj = np.dot(vec, vel) / np.dot(vec, vec) * vec
        v0 = np.linalg.norm(vel_proj) * (2 * np.heaviside(np.dot(vec, vel), 1) - 1)  # Gets speed for robot

        dist = np.linalg.norm(vec)

        t1 = (self.MAX_VELOCITY - v0) / self.ACCELERATION
        d1 = (self.MAX_VELOCITY + v0) / 2 * t1

        t2 = self.MAX_VELOCITY / self.ACCELERATION
        d2 = self.MAX_VELOCITY / 2 * t2

        if d1 + d2 < dist:  # Trapetz velocity profile
            dc = dist - (d1 + d2)
            tc = dc / self.MAX_VELOCITY

            return t1 + t2 + tc

        vp = (np.sqrt(2) * np.sqrt(v0**2 + 2 * dist * self.ACCELERATION)) / 2

        t1 = (vp - v0) / self.ACCELERATION
        t2 = vp / self.ACCELERATION

        return t1 + t2

    """
    Publishes goal pose
    """
    def _publish_pose(self, pose):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = f"{self.robot_prefix}_base_link"  # "link_base"

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
        crd_ee = np.array(self.ee_pose_rel_base_frame_theoretical[0:3])
        crd_goal = np.array(goal_pose_rel_base_frame[0:3])
        quat_ee = np.array(self.ee_pose_rel_base_frame[3:7])
        quat_goal = np.array(goal_pose_rel_base_frame[3:7])

        dist = crd_goal - crd_ee
        dist_norm = np.linalg.norm(dist)

        if dist_norm < 1e-4:
            return np.concatenate([crd_goal, goal_pose_rel_base_frame[3:7]])

        dir_vec = dist / dist_norm
        vel = np.dot(self.robot_theoretical_velocity, dir_vec)

        dt = 1.0 / self.UPDATE_RATE

        if abs(dist_norm) <= 0.5 * (vel)**2 / self.ACCELERATION:
            if dist_norm > 1e-6:  # avoid division by zero
                a = -((vel)**2) / (2 * dist_norm) * np.sign(vel)
            else:
                a = -self.ACCELERATION * np.sign(vel)
        elif abs(vel) < self.MAX_VELOCITY:
            a = self.ACCELERATION * np.sign(np.dot(dist, dir_vec))
        else:
            a = 0.0

        vel_new = vel + a * dt

        vel_new = np.clip(vel_new, -self.MAX_VELOCITY, self.MAX_VELOCITY)

        step = vel_new * dt * dir_vec
        new_crd = crd_ee + step

        if np.linalg.norm(new_crd - crd_ee_start) > np.linalg.norm(crd_goal - crd_ee_start):
            new_crd = crd_goal
            vel_new = 0.0

        self.robot_theoretical_velocity = vel_new * dir_vec

        frac = 1 - np.linalg.norm(dist) / np.linalg.norm(crd_goal - crd_ee_start)
        frac = float(np.clip(frac, 0.0, 1.0))

        quat_ee /= np.linalg.norm(quat_ee) + 1e-16
        quat_goal /= np.linalg.norm(quat_goal) + 1e-16

        if np.dot(quat_ee, quat_goal) < 0.0:
            quat_goal = -quat_goal

        r_pair = R.from_quat(np.vstack([quat_ee, quat_goal]))
        slerp = Slerp([0.0, 1.0], r_pair)
        new_quat = slerp([frac]).as_quat()[0]

        new_pose = np.concatenate([new_crd, new_quat])

        self.ee_pose_rel_base_frame_theoretical = new_pose

        return new_pose

    def _test_of_PID_controller(self):
        error = np.array(self.goal_pose_rel_base_frame[0:3]) - np.array(self.ee_pose_rel_base_frame[0:3])
        

        kp = 0.2 #0.03
        ki = 0.35  #0.03
        kd = 0.004 #0.05

        dt = 1.0 / self.UPDATE_RATE

        int_error = error * dt

        signed_gp = np.sign(self.goal_pose_velocity)
        signed_ie = np.sign(int_error)

        for axis in range(len(int_error)):
            if signed_gp[axis] != signed_ie[axis] and signed_gp[axis] != 0:
                int_error[axis] = 0 

        self.integral += int_error
        self.integral_double += self.integral * dt
        derivative = (error - self.previous_error) / dt

        output = kp * error + ki * self.integral + kd * derivative + self.goal_pose_velocity * dt
        self.previous_error = error * dt

        new_crd = np.array(self.ee_pose_rel_base_frame[0:3]) + output 

        new_pose = np.concatenate([new_crd, self.goal_pose_rel_base_frame[3:7]])
            
        self.get_logger().info(f"Output value: {output}")

        return new_pose





def main(args=None):
    rclpy.init(args=args)
    node = RelativeMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
