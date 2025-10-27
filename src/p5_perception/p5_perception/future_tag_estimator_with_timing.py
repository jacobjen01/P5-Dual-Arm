import re
import time
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from p5_interfaces.msg import Tagvector

class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')

        # Parameters
        self.declare_parameter('input_topic', '/tf')
        self.declare_parameter('output_topic', '/future_tag_poses')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('timing_window', 50)  # number of callbacks to average

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value
        self.timing_window = self.get_parameter('timing_window').get_parameter_value().integer_value

        qos = QoSProfile(depth=qos_depth)
        self.publisher = self.create_publisher(Tagvector, output_topic, qos)
        self.subscription = self.create_subscription(TFMessage, input_topic, self.tf_callback, qos)

        # Data storage
        self.tag_history = {}  # tag_key -> deque of (t, (x,y,z))
        self.tag_motion = {}   # tag_key -> motion vector/speed

        # Timing
        self.callback_times = deque(maxlen=self.timing_window)

        self.get_logger().info(f"Deque-based FutureTagEstimator initialized.")

    # -----------------------------------------------------------
    def time_to_sec(self, stamp):
        return float(stamp.sec) + float(getattr(stamp, 'nanosec', 0)) * 1e-9

    # -----------------------------------------------------------
    def compute_motion_from_history(self, history_deque):
        if len(history_deque) < 2:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        t_old, pos_old = history_deque[0]
        t_new, pos_new = history_deque[-1]
        dt = t_new - t_old
        if dt <= 0:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        vx = (pos_new[0] - pos_old[0]) / dt
        vy = (pos_new[1] - pos_old[1]) / dt
        vz = (pos_new[2] - pos_old[2]) / dt
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        if speed == 0.0:
            direction_unit = (0.0, 0.0, 0.0)
        else:
            direction_unit = (vx/speed, vy/speed, vz/speed)
        return direction_unit, (vx, vy, vz), speed

    # -----------------------------------------------------------
    def tf_callback(self, msg: TFMessage):
        if not msg.transforms:
            self.get_logger().warning("Received empty TFMessage")
            return

        start_time = time.perf_counter()

        for transform in msg.transforms:
            child = transform.child_frame_id
            if ':' in child:
                tag_key = child.split(':')[-1]  # take the part after the colon
            else:
                tag_key = child.replace('/', '_')  # fallback for other frame names

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            t = self.time_to_sec(transform.header.stamp)

            # ensure history deque exists
            if tag_key not in self.tag_history:
                self.tag_history[tag_key] = deque(maxlen=self.history_size)
            self.tag_history[tag_key].append((t, (tx, ty, tz)))

            # compute motion
            direction_unit, direction, speed = self.compute_motion_from_history(self.tag_history[tag_key])
            self.tag_motion[tag_key] = {'direction_unit': direction_unit, 'direction': direction, 'speed': speed}

            # publish
            msg_out = Tagvector()
            msg_out.tag_id = int(tag_key)
            msg_out.vx, msg_out.vy, msg_out.vz = direction
            msg_out.vx_unit, msg_out.vy_unit, msg_out.vz_unit = direction_unit
            msg_out.speed = speed
            self.publisher.publish(msg_out)

        # --------------------- Timing ---------------------
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000
        self.callback_times.append(elapsed_ms)
        avg_time = sum(self.callback_times)/len(self.callback_times)
        self.get_logger().info(f"Callback time: {elapsed_ms:.3f} ms, avg over last {len(self.callback_times)}: {avg_time:.3f} ms")


# -----------------------------------------------------------
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
