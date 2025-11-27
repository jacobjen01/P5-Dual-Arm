import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from p5_interfaces.srv import PoseConfig


class PoseConfigListener(Node):

    def __init__(self):
        super().__init__('p5_pose_config_listener')
        self.joint_values = self.create_subscription(JointState, '/joint_states', self.get_joint_val, 10)
        self.srv = self.create_service(PoseConfig, '/p5_test', self.change_pose)

    def change_pose(self, request, response): 
        response.message = True
        return response

    def get_joint_val(self, msg):
        self.get_logger().info(f'{msg.position[0]}')


def main(args=None):
    rclpy.init(args=args)

    node = PoseConfigListener()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
