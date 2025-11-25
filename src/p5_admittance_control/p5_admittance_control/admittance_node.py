"""
Admittance control node.
- Create service calls for mass, damping and stifness declaretion.
- Subscribes to force/torque sensor data
- Calculate end-effector displacement based on the dynamics.
- Get goal pose from PoseStamped topic.
- Convert displacement vector and goal pose quanteriones to transformations matrices.
- Combine both transformation matrices to get current end-effector pose.
- Convert current end-effector pose transformation matrix to pose in quaternions.
- Publisher current pose to robot_name/servo_node/pose_target_cmds topic!
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import WrenchStamped, PoseStamped
from p5_interfaces.srv import AdmittanceConfig
from p5_interfaces.srv import SaveAdmittanceParam
import json

from p5_interfaces.srv import AdmittanceShowStatus
from p5_interfaces.srv import AdmittanceSetStatus


class EEAdmittance(Node):
    def __init__(self):
        super().__init__('p5_admittance_control')
        self.robot_name = self.declare_parameter('robot_name', 'alice')
        self.robot_name = self.get_parameter('robot_name').value
        self.parameters = self.declare_parameter('parameters', 'default')
        self.parameters = self.get_parameter('parameters').value

        # Create service servers
        self.active = False
        self.config = self.create_service(
            AdmittanceConfig,
            'p5_admittance_config',
            self.change_param)
        self.save = self.create_service(
            SaveAdmittanceParam,
            'p5_save_admittance_param',
            self.save_param)
        self.status = self.create_service(
            AdmittanceShowStatus,
            'p5_admittance_show_staus',
            self.show_status)
        self.status_set = self.create_service(
            AdmittanceSetStatus,
            'p5_admittance_set_state',
            self.set_status)

        # Load parameters
        try:
            with open('admittance_param.json', 'r') as f:
                data = json.load(f)
            self.M = np.diag(data[self.parameters]['M'])
            self.D = np.diag(data[self.parameters]['D'])
            self.K = np.diag(data[self.parameters]['K'])
            self.alpha = data[self.parameters]['alpha']
        except BaseException:
            self.M = np.diag([1.5, 1.5, 1.5, 0.1, 0.1, 0.1])
            self.D = np.diag([20.0, 20.0, 20.0, 3.0, 3.0, 3.0])
            self.K = np.diag([50.0, 50.0, 50.0, 1.2, 1.2, 1.2])
            self.alpha = 0.01
        self.get_logger().info(
            f"Loaded M={np.diag(self.M)}, "
            f"D={np.diag(self.D)}, "
            f"K={np.diag(self.K)}, "
            f"alpha={self.alpha}"
        )

        self.update_rate = 250

        # State variables
        self.v_ee = np.zeros(6)               # EE velocity (x, y, z, roll, pitch, yaw)
        self.x_ee = np.zeros(6)               # EE displacement relative to goal reference
        self.x_goal = np.zeros(7)             # goal pose in quantariones (x, y, z, qx, qy, qz, qw)
        self.wrench = np.zeros(6)                  # latest measured wrench
        self.goal_received = False

        self.robot_force = self.create_subscription(WrenchStamped,
                                                    f'/{self.robot_name}_force_torque_sensor_broadcaster/wrench',
                                                    self.wrench_cb, 10)

        self.robot_goal = self.create_subscription(PoseStamped,
                                                   'p5_robot_pose_to_admittance',
                                                   self.goal_cb, 10)

        self.current_goal_pub = self.create_publisher(PoseStamped,
                                                      'p5_robot_pose_before_safety', 10)

        # Timer for control loop
        self.timer = self.create_timer(1 / self.update_rate, self.control_loop)

    def change_param(self, request, response):
        self.M = np.diag(request.m)
        self.D = np.diag(request.d)
        self.K = np.diag(request.k)
        self.alpha = request.alpha
        response.message = True
        return response

    def save_param(self, request, response):
        try:
            with open("admittance_param.json", "r") as f:
                data = json.load(f)
        except BaseException:
            data = {}
        data[request.param_name] = {
            "M": np.diag(self.M).tolist(),
            "D": np.diag(self.D).tolist(),
            "K": np.diag(self.K).tolist(),
            "alpha": self.alpha
        }
        json_str = json.dumps(data, indent=4)
        with open("admittance_param.json", "w") as f:
            f.write(json_str)

        response.message = True

        return response

    def show_status(self, request, response):
        response.active = self.active
        response.robot_name = self.robot_name
        response.m_parameter = np.diag(self.M)
        response.d_parameter = np.diag(self.D)
        response.k_parameter = np.diag(self.K)
        response.alpha_parameter = self.alpha
        response.update_rate = self.update_rate
        return response

    def set_status(self, request, response):
        self.active = request.active
        self.update_rate = request.update_rate
        response.message = True
        return response

    def wrench_cb(self, msg: WrenchStamped):
        # Extract force and torque
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z
        # Store as numpy vector for admittance law
        FT_vector = np.array([fx, fy, fz, tx, ty, tz])
        # self.get_logger().info(f"Force {fx, fy, fz}, Torque: {tx, ty, tz}")
        # self.wrench = FT_vector

        # Low-pass filter
        self.wrench = self.alpha * FT_vector + (1 - self.alpha) * self.wrench
        # self.get_logger().info(f"Force {self.wrench}")

    def goal_cb(self, msg: PoseStamped):
        self.goal_frame = msg.header.frame_id
        # Update goal pose from PoseStamped message (posittion + quaternions)
        self.x_goal[0] = msg.pose.position.x
        self.x_goal[1] = msg.pose.position.y
        self.x_goal[2] = msg.pose.position.z
        self.x_goal[3] = msg.pose.orientation.x
        self.x_goal[4] = msg.pose.orientation.y
        self.x_goal[5] = msg.pose.orientation.z
        self.x_goal[6] = msg.pose.orientation.w
        self.goal_received = True
        # self.get_logger().info(f"Goal position {self.x_goal}")

    def update_admittance(self):
        # Admittance control law: M * dv/dt + D * v + K * x = F
        # Discretized: v[k+1] = v[k] + dt * M^-1 * (F - D*v[k] - K*x[k])
        #              x[k+1] = x[k] + dt * v[k+1]

        M_inv = np.linalg.inv(self.M)
        a_ee = M_inv @ (self.wrench - self.D @ self.v_ee - self.K @ self.x_ee)
        indices = np.where(a_ee[:3] > 1.5)[0]
        a_ee[indices] = 1.5

        # Update velocity and position  (forward Euler integration)
        dt = 1 / self.update_rate
        self.v_ee += a_ee * dt
        indices = np.where(self.v_ee[:3] > 0.6)[0]
        self.v_ee[indices] = 0.6
        self.x_ee += self.v_ee * dt

    def get_TM_displacement(self):
        # Create transformation matrix for position displacements of EE based on e_xx.
        r = R.from_euler('zxy', self.x_ee[3:6])
        r.as_matrix()
        T_delta = np.vstack((np.hstack((r.as_matrix(), [[self.x_ee[0]],
                                                        [self.x_ee[1]], [self.x_ee[2]]])), [0, 0, 0, 1]))
        return T_delta

    def get_TM_goal(self):
        # Create transformation matrix for goal pose of EE based on x_goal quarternions.
        r = R.from_quat(self.x_goal[3:7])
        r.as_matrix()
        T_goal = np.vstack((np.hstack((r.as_matrix(), [[self.x_goal[0]],
                                                       [self.x_goal[1]], [self.x_goal[2]]])), [0, 0, 0, 1]))
        return T_goal

    def get_TM_current(self):
        # Multiply goal pose transformation matrix by displacement transformation matrix
        # based on admittance control.
        T_delta = self.get_TM_displacement()
        T_goal = self.get_TM_goal()
        if self.active == True:
            T_current = T_goal @ T_delta
        else:
            T_current = T_goal
        return T_current

    def convert_TM_to_pose(self):
        T = self.get_TM_current()
        position = T[0:3, 3]
        r = R.from_matrix(T[0:3, 0:3])
        quat = r.as_quat()  # x, y, z, w
        pose = np.hstack((position, quat))
        return pose

    def control_loop(self):
        # Only run admittance if we have a goal
        if not self.goal_received:
            return
        self.update_admittance()
        # Get current pose in quaternions.
        current_pose = self.convert_TM_to_pose()

        # Publish current pose to the 'robot_pose_before_safety' topic.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.goal_frame

        pose_msg.pose.position.x = float(current_pose[0])
        pose_msg.pose.position.y = float(current_pose[1])
        pose_msg.pose.position.z = float(current_pose[2])
        pose_msg.pose.orientation.x = float(current_pose[3])
        pose_msg.pose.orientation.y = float(current_pose[4])
        pose_msg.pose.orientation.z = float(current_pose[5])
        pose_msg.pose.orientation.w = float(current_pose[6])

        self.current_goal_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EEAdmittance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
