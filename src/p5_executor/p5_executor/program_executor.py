import rclpy
import numpy as np

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


class ProgramExecutor(Node):
    def __init__(self):
        super().__init__('program_executor')


def main(args=None):
    rclpy.init(args=args)
    node = ProgramExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print('Hi from p5_executor.')


if __name__ == '__main__':
    main()
