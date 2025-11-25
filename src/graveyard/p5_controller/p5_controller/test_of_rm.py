import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import String
from p5_interfaces.srv import MoveToPose, SetLinearMovement, SetReferenceFrame


class TestNodeForBase(Node):

    def __init__(self):
        super().__init__('test_of_rm')

        self.cli = self.create_client(MoveToPose, 'move_to_pose')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = MoveToPose.Request()

    def send_request(self):
        # pose = [0.3, -0.5, 0.4, 1.0, 0.0, 0.0, 0.0]
        # pose = [0.3, 0.5, 0.4, 1.0, 0.0, 0.0, 0.0]
        pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.req.pose.position.x = pose[0]
        self.req.pose.position.y = pose[1]
        self.req.pose.position.z = pose[2]
        self.req.pose.orientation.x = pose[3]
        self.req.pose.orientation.y = pose[4]
        self.req.pose.orientation.z = pose[5]
        self.req.pose.orientation.w = pose[6]

        self.req.linear = True
        self.req.use_tracking_velocity = True
        self.req.frame = "tag36h11:1"

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    minimal_client = TestNodeForBase()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(f'Result: {response.resp}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
