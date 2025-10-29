#!/usr/bin/env python3
"""
Admittance control node.
- Defines virtual dynamics constants (mass, damping and stiffness).
- Subscribes to force/torque sensor data
- Calculate end-effector displacement based on the dynamics.
- Get goal pose from PoseStamped topic.
- Convert displacement vector and goal pose quanteriones to transformations matrices.
- Combine both transformation matrices to get current end-effector pose.
- Convert current end-effector pose transformation matrix to pose in quaternions.
- Publisher current pose to robot_pose_before_safety topic!
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import WrenchStamped, PoseStamped


class EEAdmittance(Node):
    def __init__(self):
        super().__init__('ee_admittance')

        # Robot name parameter.
        self.declare_parameter('robot_name', 'alice')   # default to 'alice'
        self.robot_name = self.get_parameter('robot_name').value

        # Parameters (tunable virtual dynamics)
        self.declare_parameter('M', [1.5, 1.5, 1.5, 0.1, 0.1, 0.1])   # virtual mass
        self.declare_parameter('D', [20.0, 20.0, 20.0, 3.0, 3.0, 3.0])   # damping
        self.declare_parameter('K', [50.0, 50.0, 50.0, 1.2, 1.2, 1.2])      # stiffness
        self.declare_parameter('rate_hz', 250.0)                # control loop rate

        # Load parameters
        self.M = np.diag(self.get_parameter('M').value)
        self.D = np.diag(self.get_parameter('D').value)
        self.K = np.diag(self.get_parameter('K').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)

        # State variables
        self.v_ee = np.zeros(6)               # EE velocity (x, y, z, roll, pitch, yaw)
        self.x_ee = np.zeros(6)               # EE displacement relative to goal reference
        self.x_goal = np.zeros(7)             # goal pose in quantariones (x, y, z, qx, qy, qz, qw)
        self.wrench = np.zeros(6)                  # latest measured wrench
        self.goal_received = False
        # self.x_current = np.zeros(7)        # current EE pose (x, y, z, qx, qy, qz, qw)

        self.robot_force = self.create_subscription(WrenchStamped, f'/{self.robot_name}_force_torque_sensor_broadcaster/wrench',
                                                    self.wrench_cb, 10)

        self.robot_goal = self.create_subscription(PoseStamped, f'/robot_pose_to_admittance_{self.robot_name}',
                                                   self.goal_cb, 10)

        self.current_goal_pub = self.create_publisher(
            PoseStamped, f'{self.robot_name}/servo_node/pose_target_cmds', 10)

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

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
        #self.wrench = FT_vector

        # Low-pass filter could be added here
        alpha = 0.01    # filter coefficient
        self.wrench = alpha * FT_vector + (1 - alpha) * self.wrench
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

        # Update velocity and position  (forward Euler integration)
        self.v_ee += a_ee * self.dt
        self.x_ee += self.v_ee * self.dt

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
        T_current = T_goal @ T_delta
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
        pose_msg.header.frame_id = 'world'

        pose_msg.pose.position.x = float(current_pose[0])
        pose_msg.pose.position.y = float(current_pose[1])
        pose_msg.pose.position.z = float(current_pose[2])
        pose_msg.pose.orientation.x = float(current_pose[3])
        pose_msg.pose.orientation.y = float(current_pose[4])
        pose_msg.pose.orientation.z = float(current_pose[5])
        pose_msg.pose.orientation.w = float(current_pose[6])

        self.current_goal_pub.publish(pose_msg)
        self.get_logger().info(f'Current position: {pose_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = EEAdmittance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
