#!/usr/bin/env python3
import time

import rclpy
import math
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController

class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager_client')
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /controller_manager/switch_controller...')

    def switch_controller(self, controllers_to_stop):
        request = SwitchController.Request()
        request.deactivate_controllers = controllers_to_stop
        request.strictness = 2  # Use STRICT
        request.activate_asap = True  # Activate the new controller as soon as possible

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully switched controllers: {controllers_to_stop} ')
        else:
            self.get_logger().error('Failed to call service: %r' % future.exception())

def switch_to_joint_control():
    controller_manager = ControllerManager()

    controllers_to_stop = ['alice_forward_position_controller']
    controller_manager.switch_controller(controllers_to_stop)

    controllers_to_stop = ['bob_forward_position_controller']
    controller_manager.switch_controller(controllers_to_stop)

    controller_manager.destroy_node()

def switch_to_position_control():
    controller_manager = ControllerManager()

    controllers_to_stop = ['alice_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop)

    controllers_to_stop = ['bob_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop)

    controller_manager.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    switch_to_position_control()
    switch_to_joint_control()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
