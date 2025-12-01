from scipy.io import savemat
import rclpy

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from datetime import datetime

"""
Simple notes:
Start logging: 
ros2 service call /start_logging std_srvs/srv/Trigger "{}"

Stop logging:
ros2 service call /stop_logging std_srvs/srv/Trigger "{}"
"""


class EndEffectorFrameLogger(Node):
    def __init__(self):
        super().__init__('end_effector_frame_logger')

        self.PATH = "datalog/"
        self.data = {"pose_alice": [], "pose_bob": [], "pose_target": []}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'map'
        self.source_frame = 'base_link'

        self.logging_active = False
        self.timer = None

        # Callback group for services
        cb_group = ReentrantCallbackGroup()

        # Create services
        self.start_service = self.create_service(
            Trigger, 'start_logging', self.start_logging, callback_group=cb_group
        )
        self.stop_service = self.create_service(
            Trigger, 'stop_logging', self.stop_logging, callback_group=cb_group
        )

    def start_logging(self, request, response):
        if not self.logging_active:
            self.logging_active = True
            self.data = {"pose_alice": [], "pose_bob": [], "pose_target": []}
            self.timer = self.create_timer(0.05, self.data_logging)
            self.get_logger().info("Data logging started.")
            response.success = True
            response.message = "Logging started."
        else:
            response.success = False
            response.message = "Logging already active."
        return response

    def stop_logging(self, request, response):
        if self.logging_active:
            self.logging_active = False
            if self.timer:
                self.timer.cancel()
                self.timer = None
            self.save_matlab()
            self.get_logger().info("Data logging stopped and saved.")
            response.success = True
            response.message = "Logging stopped and data saved."
        else:
            response.success = False
            response.message = "Logging was not active."
        return response

    def data_logging(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("alice_base_link", "alice_tool0", now)
            crd = trans.transform.translation
            quat = trans.transform.rotation
            alice_pose = [crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w]

            trans = self.tf_buffer.lookup_transform("bob_base_link", "bob_tool0", now)
            crd = trans.transform.translation
            quat = trans.transform.rotation
            bob_pose = [crd.x, crd.y, crd.z, quat.x, quat.y, quat.z, quat.w]

            self.data["pose_alice"].append(alice_pose)
            self.data["pose_bob"].append(bob_pose)

            self.get_logger().info("Logging data...")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def save_matlab(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.PATH}{timestamp}.mat"
        savemat(filename, self.data)
        self.get_logger().info(f"Data saved to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorFrameLogger()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
