import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Pose, String
from p5_interfaces.srv import MoveToPose, SetLinearMovement, SetReferenceFrame
from p5_safety._error_handling import ErrorHandler


class RelativeMover(Node):
    def __init__(self):
        super().__init__('relative_mover_node')

        self.error_handler = ErrorHandler(self)

        self.declare_parameter('robot_prefix', 'alice')
        self.robot_prefix = self.get_parameter('robot_prefix').get_parameter_value().string_value

        self.linear_movement = False
        self.linear_movement_time = 0
        self.reference_frame = None
        self.goal_pose = [0, 0, 0, 0, 0, 0, 1]


        self.pose_publisher = self.create_publisher(Pose, f"{self.robot_prefix}_robot_pose_for_admittance_control", 10)

        self.set_linear_movement_service = self.create_service(SetLinearMovement, 'set_linear_movement', self.set_linear_movement_callback)
        self.move_to_pose_service = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)
        self.set_tf_tree_service = self.create_service(SetReferenceFrame, 'set_reference_frame', self.set_reference_frame_callback)

    def set_linear_movement_callback(self, request, response):
        try:
            self.linear_movement = request.linear
            self.linear_movement_time = request.time

            response.resp = True
            return response

        except Exception as e:
            self.getlogger().error(e)
            self.error_handler.report_error(self.error_handler.fatal, f"Failed to send true response. Error: {e}")

            response.resp = False
            return response





def main(args=None):
    rclpy.init(args=args)
    node = RelativeMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
