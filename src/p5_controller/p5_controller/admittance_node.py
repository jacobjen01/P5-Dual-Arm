#!/usr/bin/env python3
"""
Admittance control node.
- Subscribes to force/torque sensor data
- Publisher pose to MoveIt!
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import WrenchStamped, PoseStamped


class EEAdmittance(Node):
    def __init__(self):
        super().__init__('ee_admittance')

        # Parameters (tunable virtual dynamics)
        self.declare_parameter('M', [5.0, 5.0, 5.0, 0.5, 0.5, 0.5])   # virtual mass
        self.declare_parameter('D', [100.0, 100.0, 100.0, 5.0, 5.0, 5.0])   # damping
        self.declare_parameter('K', [50.0, 50.0, 50.0, 2.0, 2.0, 2.0])      # stiffness
        self.declare_parameter('rate_hz', 100.0)                # control loop rate

        self.ref_initialized = False   # new flag to track reference initialization

        # Load parameters
        self.M = np.diag(self.get_parameter('M').value)
        self.D = np.diag(self.get_parameter('D').value)
        self.K = np.diag(self.get_parameter('K').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)

        # State variables
        self.v_ee = np.zeros(6)                    # EE velocity (x, y, z, roll, pitch, yaw)
        self.x_ee = np.zeros(6)                    # EE displacement relative to reference
        self.x_ref = np.zeros(6)                   # reference pose
        self.wrench = np.zeros(6)                  # latest measured wrench
        self.x_ee = np.array([1.0, 4.8, 7.4, 0.6, -0.5, -0.2])

        self.alice_force = self.create_subscription(WrenchStamped, '/alice_force_torque_sensor_broadcaster/wrench',
                                                    self.wrench_cb, 10)
        self.bob_force = self.create_subscription(WrenchStamped, '/bob_force_torque_sensor_broadcaster/wrench',
                                                  self.wrench_cb, 10)

        self.alice_goal = self.create_subscription(PoseStamped, '/robot_pose_to_admittance_alice',
                                                   self.alice_goal_cb, 10)
        # self.bob_goal = self.create_subscription(PoseStamped, '/robot_pose_to_admittance_bob',
        #                                         self.bob_goal_cb, 10)

    def wrench_cb(self, msg: WrenchStamped):
        # Extract force and torque
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z

        # Store as numpy vector for admittance law
        self.wrench = np.array([fx, fy, fz, tx, ty, tz])

    def alice_goal_cb(self, msg: PoseStamped):
        # On first goal message, initialize reference pose
        self.x_ref[0] = msg.pose.position.x
        self.x_ref[1] = msg.pose.position.y
        self.x_ref[2] = msg.pose.position.z
        self.x_ref[3] = msg.pose.orientation.x
        self.x_ref[4] = msg.pose.orientation.y
        self.x_ref[5] = msg.pose.orientation.z

    def update_admittance(self):
        # Admittance control law: M * dv/dt + D * v + K * x = F
        # Discretized: v[k+1] = v[k] + dt * M^-1 * (F - D*v[k] - K*x[k])
        #              x[k+1] = x[k] + dt * v[k+1]

        M_inv = np.linalg.inv(self.M)
        a_ee = M_inv @ (self.wrench - self.D @ self.v_ee - self.K @ self.x_ee)

        # Update velocity and position  (forward Euler integration)
        self.v_ee += a_ee * self.dt
        self.x_ee += self.v_ee * self.dt

    def get_transform_matrix_ee(self):
        # Create transformation matrix for position displacements of EE based on e_xx.
        Rot_x = np.array([
            [1, 0, 0],
            [0, np.cos(self.x_ee[3]), -np.sin(self.x_ee[3])],
            [0, np.sin(self.x_ee[3]), np.cos(self.x_ee[3])]
        ])

        Rot_y = np.array([
            [np.cos(self.x_ee[4]), 0, np.sin(self.x_ee[4])],
            [0, 1, 0],
            [-np.sin(self.x_ee[4]), 0, np.cos(self.x_ee[4])]
        ])

        Rot_z = np.array([
            [np.cos(self.x_ee[5]), -np.sin(self.x_ee[5]), 0],
            [np.sin(self.x_ee[5]), np.cos(self.x_ee[5]), 0],
            [0, 0, 1]
        ])

        Rot_ee = Rot_z @ Rot_y @ Rot_x

        Trans_ee = np.array([[self.x_ee[0]], [self.x_ee[1]], [self.x_ee[2]]])

        T_delta_ee = np.vstack((np.hstack((Rot_ee, Trans_ee)), [0, 0, 0, 1]))

        return T_delta_ee


def main(args=None):
    rclpy.init(args=args)
    node = EEAdmittance()
    print(node.M)
    T = node.get_transform_matrix_ee()
    print(T)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
