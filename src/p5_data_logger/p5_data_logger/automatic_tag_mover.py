from scipy.io import savemat
import rclpy

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TransformStamped, PoseStamped
from std_srvs.srv import Trigger
from datetime import datetime
import random


class AutomaticTagMover(Node):
    def __init__(self):
        super().__init__('auto_tag_mover')

        self.VELOCITY = [-0.1, 0.0, 0.0]
        self.UPDATE_RATE = 30

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose = [0.3, -0.8, 0.9, 0.0, -0.707, -0.707, 0.0]

        self.timer = self.create_timer(1/self.UPDATE_RATE, self.move_tag)

    def move_tag(self):
        df = 1.0 / self.UPDATE_RATE
        self.pose[0] += self.VELOCITY[0] * df
        self.pose[1] += self.VELOCITY[1] * df
        self.pose[2] += self.VELOCITY[2] * df

        #if self.pose[0] > -0.05:
            #self.VELOCITY[0] = -0.1
        #    self.pose[0] = -0.6

        #if self.pose[0] < -0.6:
        #    self.VELOCITY[0] = 0.1


        self.create_current_goal_frame()

    def create_current_goal_frame(self):
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "mir"
            t.child_frame_id = f"camera_color_optical_frame"

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_color_optical_frame"
            t.child_frame_id = f"tag36h11:5"

            t.transform.translation.x = self.pose[0] #+ random.uniform(-0.01, 0.01)
            t.transform.translation.y = self.pose[1] #+ random.uniform(-0.01, 0.01)
            t.transform.translation.z = self.pose[2] #+ random.uniform(-0.01, 0.01)

            t.transform.rotation.x = self.pose[3]
            t.transform.rotation.y = self.pose[4]
            t.transform.rotation.z = self.pose[5]
            t.transform.rotation.w = self.pose[6]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Failed to create transform {e}')



def main(args=None):
    rclpy.init(args=args)
    node = AutomaticTagMover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
