import rclpy
import numpy as np
#from p5_safety._error_handling import ErrorHandler

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener

class RobotProtectiveStop(Node):
    def __init__(self):
        super().__init__('robot_protective_stop')

        #self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub_a = self.create_subscription(
            PoseStamped,
            '/robot_pose_before_safety_alice',
            self.pose_alice_callback,
            10)
        self.pose_sub_b = self.create_subscription(
            PoseStamped,
            '/robot_pose_before_safety_bob',
            self.pose_bob_callback,
            10)
        self.protective_stop_sub = self.create_subscription(
            Bool,
            '/protective_stop',
            self.protective_stop_callback,
            10)

        self.pose_pub_alice = self.create_publisher(PoseStamped, '/Moveit_servo_robot_alice_pose', 10)
        self.pose_pub_bob = self.create_publisher(PoseStamped, '/Moveit_servo_robot_bob_pose', 10)

        self.protective_stop_active = False
        self.pose_alice = None
        self.protective_stop_alice = None
        self.protective_stop_bob = None

        #self.timer = self.create_timer(0.1, self.timer_callback)  

        self.get_logger().info("Robot Protective Stop node started.")


    def _get_tf_tree_pose(self, parent_name, child_name):
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(parent_name, child_name, now)

                pose = np.array([trans.transform.translation.x, trans.transform.translation.y,
                                trans.transform.translation.z, trans.transform.rotation.x, 
                                trans.transform.rotation.y, trans.transform.rotation.z,
                                trans.transform.rotation.w])

                return pose

            except Exception as e:
                self.get_logger().error(f'Failed to get transform from {parent_name}/{child_name}: {e}')
                #self.error_handler.report_error(self.error_handler.info, f'Failed to get transform from {parent_name}/{child_name}: {e}')


                return []
        
    def pose_alice_callback(self, msg: PoseStamped):
        pose = msg

        if self.protective_stop_active:
            pose.pose.position.x = self.protective_stop_alice[0]
            pose.pose.position.y = self.protective_stop_alice[1]
            pose.pose.position.z = self.protective_stop_alice[2]
            pose.pose.orientation.x = self.protective_stop_alice[3]
            pose.pose.orientation.y = self.protective_stop_alice[4]
            pose.pose.orientation.z = self.protective_stop_alice[5]
            pose.pose.orientation.w = self.protective_stop_alice[6]


        self.pose_pub_alice.publish(pose)

    def pose_bob_callback(self, msg: PoseStamped):
        pose = msg

        if self.protective_stop_active:
            pose.pose.position.x = self.protective_stop_bob[0]
            pose.pose.position.y = self.protective_stop_bob[1]
            pose.pose.position.z = self.protective_stop_bob[2]
            pose.pose.orientation.x = self.protective_stop_bob[3]
            pose.pose.orientation.y = self.protective_stop_bob[4]
            pose.pose.orientation.z = self.protective_stop_bob[5]
            pose.pose.orientation.w = self.protective_stop_bob[6]


        self.pose_pub_bob.publish(pose)



    def protective_stop_callback(self, msg: Bool,):
        if msg.data and not self.protective_stop_active:
            self.protective_stop_active = True
            self.get_logger().warn("Protective stop triggered! Freezing current TF pose permanently.")
            #self.error_handler.report_error(self.error_handler.info, f'Protective stop triggered! Freezing current TF pose permanently.')

            self.protective_stop_alice = self._get_tf_tree_pose("alice_base_frame", "alice_end_effector_frame") 
            self.protective_stop_bob = self._get_tf_tree_pose("bob_base_frame", "bob_end_effector_frame")

def main(args=None):
    rclpy.init(args=args)
    node = RobotProtectiveStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


