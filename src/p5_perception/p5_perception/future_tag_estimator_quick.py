import re
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from p5_interfaces.msg import Tagvector


# ===============================================================
# AprilTagPoseBuffer — rolling pose history per tag
# ===============================================================
class AprilTagPoseBuffer:
    """Efficient rolling buffer for storing the last N (x, y, z) poses per tag."""

    def __init__(self, pose_history=5, max_tags=5):
        self.pose_history = pose_history
        self.max_tags = max_tags
        self.poses = np.zeros((max_tags, pose_history, 3))
        self.pose_idx = np.zeros(max_tags, dtype=int)
        self.tag_to_index = {}
        self.next_free_index = 0

    def add_pose(self, tag_id: int, xyz: np.ndarray):
        """Add a new pose for a specific tag."""
        if tag_id not in self.tag_to_index:
            if self.next_free_index >= self.max_tags:
                raise ValueError("Maximum number of tags reached")
            self.tag_to_index[tag_id] = self.next_free_index
            self.next_free_index += 1

        idx = self.tag_to_index[tag_id]
        i = self.pose_idx[idx] % self.pose_history
        self.poses[idx, i] = xyz
        self.pose_idx[idx] += 1

    def get_history(self, tag_id: int) -> np.ndarray:
        """Return all stored poses for a tag (oldest → newest)."""
        if tag_id not in self.tag_to_index:
            return np.array([])

        idx = self.tag_to_index[tag_id]
        n = min(self.pose_idx[idx], self.pose_history)
        start = self.pose_idx[idx] % self.pose_history
        return np.concatenate((self.poses[idx, start:], self.poses[idx, :start]))[-n:]

    def get_latest(self, tag_id: int) -> np.ndarray:
        """Return the most recent pose for a tag."""
        history = self.get_history(tag_id)
        return history[-1] if len(history) > 0 else np.array([])


# ===============================================================
# ROS2 Node — FutureTagEstimator
# ===============================================================
class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')

        # Declare parameters
        self.declare_parameter('input_topic', '/tf')
        self.declare_parameter('output_topic', '/future_tag_poses')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('max_tags', 5)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        history_size = self.get_parameter('history_size').get_parameter_value().integer_value
        max_tags = self.get_parameter('max_tags').get_parameter_value().integer_value

        qos = QoSProfile(depth=qos_depth)
        self.publisher = self.create_publisher(Tagvector, output_topic, qos)
        self.subscription = self.create_subscription(TFMessage, input_topic, self.tf_callback, qos)

        # Replace deques with our new buffer
        self.pose_buffer = AprilTagPoseBuffer(pose_history=history_size, max_tags=max_tags)

        # For timing
        self.tag_timestamps = {}  # tag_id → deque of timestamps (simple list)
        self.tag_motion = {}      # tag_id → motion vector/speed

        self.get_logger().info(f"FutureTagEstimator using AprilTagPoseBuffer ({max_tags} tags, history={history_size})")

    # -----------------------------------------------------------

    def time_to_sec(self, stamp):
        return float(stamp.sec) + float(getattr(stamp, 'nanosec', 0)) * 1e-9

    # -----------------------------------------------------------

    def compute_motion_from_history(self, timestamps, positions):
        """
        Compute velocity and direction based on oldest and newest pose.
        timestamps: list or np.array of times
        positions: np.array of shape (N, 3)
        """
        if len(positions) < 2 or len(timestamps) < 2:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0

        t_old, t_new = timestamps[0], timestamps[-1]
        pos_old, pos_new = positions[0], positions[-1]
        dt = t_new - t_old
        if dt <= 0:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0

        vx, vy, vz = (pos_new - pos_old) / dt
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed == 0.0:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        return (vx/speed, vy/speed, vz/speed), (vx, vy, vz), speed

    # -----------------------------------------------------------

    def tf_callback(self, msg: TFMessage):
        if not msg.transforms:
            self.get_logger().warning("Received empty TFMessage")
            return

        for transform in msg.transforms:
            child = transform.child_frame_id
            if ':' in child:
                tag_key = child.split(':')[-1]  # take the part after the colon
            else:
                tag_key = child.replace('/', '_')  # fallback for other frame names

            # translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # timestamp
            t = self.time_to_sec(transform.header.stamp)

            # update pose buffer
            self.pose_buffer.add_pose(tag_key, np.array([tx, ty, tz]))

            # update timestamp history (simple list)
            if tag_key not in self.tag_timestamps:
                self.tag_timestamps[tag_key] = []
            timestamps = self.tag_timestamps[tag_key]
            timestamps.append(t)
            if len(timestamps) > self.pose_buffer.pose_history:
                timestamps.pop(0)

            # compute motion
            positions = self.pose_buffer.get_history(tag_key)
            direction_unit, direction, speed = self.compute_motion_from_history(timestamps, positions)
            self.tag_motion[tag_key] = {'direction_unit': direction_unit, 'direction': direction, 'speed': speed}

            # publish
            msg_out = Tagvector()
            msg_out.tag_id = int(tag_key)
            msg_out.vx, msg_out.vy, msg_out.vz = direction
            msg_out.vx_unit, msg_out.vy_unit, msg_out.vz_unit = direction_unit
            msg_out.speed = speed
            self.publisher.publish(msg_out)

            self.get_logger().info(f"Tag {tag_key} motion -> dir: ({direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}), speed: {speed:.3f} m/s")

# ===============================================================

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
