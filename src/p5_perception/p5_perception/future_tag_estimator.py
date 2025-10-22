import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from collections import deque
import math
from p5_interfaces.msg import Tagvector

class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')
        # parameters (can be overridden via ros2 param or launch)
        self.declare_parameter('input_topic', '/tf')
        self.declare_parameter('output_topic', '/future_tag_poses')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 5)  # number of samples to keep for motion calc

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value

        qos = QoSProfile(depth=qos_depth)

        self.publisher = self.create_publisher(Tagvector, output_topic, qos)
        
        self.subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self.tf_callback,
            qos
        )

        self.tag_poses = {}      # Dictionary to store latest tag poses
        self.tag_history = {}    # { tag_key: deque([ (t, (x,y,z)), ... ], maxlen=history_size) }
        self.tag_motion = {}     # { tag_key: {'direction': (dx,dy,dz), 'speed': s} }

        #self.get_logger().info(f'Subscribed to: {input_topic} -> Publishing to: {output_topic}')

    def time_to_sec(self, stamp):
        # stamp is builtin_interfaces.msg.Time with sec/nanosec fields
        return float(stamp.sec) + float(getattr(stamp, 'nanosec', 0)) * 1e-9
    
    def compute_motion_from_history(self, history_deque):
        # history_deque contains tuples (t, (x,y,z)), oldest -> newest
        if len(history_deque) < 2:
            return (0.0, 0.0, 0.0), 0.0
        t_old, pos_old = history_deque[0]
        t_new, pos_new = history_deque[-1]
        dt = t_new - t_old
        if dt <= 0:
            return (0.0, 0.0, 0.0), 0.0
        vx = (pos_new[0] - pos_old[0]) / dt
        vy = (pos_new[1] - pos_old[1]) / dt
        vz = (pos_new[2] - pos_old[2]) / dt
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        if speed == 0.0:
            direction = (0.0, 0.0, 0.0)
        else:
            direction = (vx/speed, vy/speed, vz/speed)
        return direction, speed

    def tf_callback(self, msg: TFMessage):
        if not msg.transforms:
            self.get_logger().warning("Received empty TFMessage")
            return

        for transform in msg.transforms:
            child = transform.child_frame_id
            # try to extract numeric id (e.g. "tag_1" -> "1"), otherwise use whole name
            m = re.search(r'(\d+)', child)
            tag_key = m.group(1) if m else child.replace('/', '_')
            # assign values to variables
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            self.tag_poses[tag_key] = {
                'translation': (tx, ty, tz),
                'rotation': (qx, qy, qz, qw)
            }
        
            # timestanmp to seconds
            stamp = transform.header.stamp
            t=self.time_to_sec(stamp)

            # ensure history deques exist
            if tag_key not in self.tag_history:
                self.tag_history[tag_key] = deque(maxlen=self.history_size)
            self.tag_history[tag_key].append((t, self.tag_poses[tag_key]['translation']))


            # compute motion using up to last `history_size` samples
            direction, speed = self.compute_motion_from_history(self.tag_history[tag_key])
            self.tag_motion[tag_key] = {'direction': direction, 'speed': speed}

            # optional short log per tag
            self.get_logger().info(f"Tag {tag_key} motion -> dir: {direction}, speed: {speed:.4f} m/s")

        for tag_key in self.tag_motion:
            msg = Tagvector()
            msg.tag_id = int(tag_key)
            msg.vx = self.tag_motion[tag_key]['direction'][0]
            msg.vy = self.tag_motion[tag_key]['direction'][1]
            msg.vz = self.tag_motion[tag_key]['direction'][2]
            self.publisher.publish(msg)

        #self.get_logger().info(f"Current tag poses: {self.tag_poses}")

def main(args=None):
    rclpy.init(args=args)
    node = FutureTagEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()