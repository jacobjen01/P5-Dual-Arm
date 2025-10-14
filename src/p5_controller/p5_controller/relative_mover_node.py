import rclpy

from rclpy.node import Node

class RelativeMover(Node):
    def __init__(self):
        super().__init__('relative_mover_node')

        self.declare_parameter('robot_prefix', 'alice')
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value



def main(args=None):
    rclpy.init(args=args)
    node = RelativeMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
