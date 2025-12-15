import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from collections import deque
import math
import numpy as np
import time

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from std_msgs.msg import Header

from p5_interfaces.msg import TagvectorArray, Tagvector
from p5_safety._error_handling import ErrorHandler
from rclpy.qos import QoSProfile


class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')

        # TF
        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.error_handler = ErrorHandler(self)

        # ---------------- Parameters ----------------
        self.declare_parameter('input_topic', '/tf')
        self.declare_parameter('output_topic', '/future_tag_vector')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 10)
        self.declare_parameter('median_window', 5)

        # Alpha–Beta filter parameters
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('beta', 0.1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        qos_depth = self.get_parameter('qos_depth').value

        self.history_size = self.get_parameter('history_size').value
        self.median_window = self.get_parameter('median_window').value

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value

        qos = QoSProfile(depth=qos_depth)

        # ---------------- ROS I/O ----------------
        self.tagvector_publisher = self.create_publisher(
            TagvectorArray, output_topic, qos
        )

        self.subscription = self.create_subscription(
            TFMessage, input_topic, self.tf_callback, qos
        )

        # ---------------- State ----------------
        self.ab_state = {}          # tag_key → {pos, vel, t}
        self.est_tag_pos = {}       # tag_key → deque[(t, pose)]

    # --------------------------------------------------
    # Utility
    # --------------------------------------------------
    def time_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def create_tf_tree(self, parent, child, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]

        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]

        self.tf_broadcaster.sendTransform(t)

    def get_tf_tree_crd(self, parent, child):
        try:
            trans = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ], dtype=float)
        except Exception as e:
            self.error_handler.report_error(
                self.error_handler.info,
                f'TF lookup failed {parent}->{child}: {e}'
            )
            return None

    # --------------------------------------------------
    # Alpha–Beta Filter
    # --------------------------------------------------
    def alpha_beta_update(self, tag_key, measurement, t):
        if tag_key not in self.ab_state:
            self.ab_state[tag_key] = {
                'pos': measurement.copy(),
                'vel': np.zeros(3),
                't': t
            }
            return measurement, np.zeros(3)

        state = self.ab_state[tag_key]
        dt = t - state['t']

        if dt <= 1e-6:
            return state['pos'], state['vel']

        # Prediction
        x_pred = state['pos'] + state['vel'] * dt
        v_pred = state['vel']

        # Residual
        r = measurement - x_pred

        # Update
        x_new = x_pred + self.alpha * r
        v_new = v_pred + (self.beta / dt) * r

        state['pos'] = x_new
        state['vel'] = v_new
        state['t'] = t

        return x_new, v_new

    # --------------------------------------------------
    # Median filter on estimated poses
    # --------------------------------------------------
    def median_pose_from_est(self, tag_key, child):
        if self.median_window <= 1:
            return

        hist = self.est_tag_pos.get(tag_key, [])
        if len(hist) == 0:
            return

        window = min(self.median_window, len(hist))
        recent = np.array([p[1] for p in list(hist)[-window:]])

        med_t = np.median(recent[:, 0:3], axis=0)
        med_q = np.median(recent[:, 3:7], axis=0)

        norm = np.linalg.norm(med_q)
        if norm > 1e-12:
            med_q /= norm
        else:
            med_q = recent[-1, 3:7]

        pose = (
            float(med_t[0]), float(med_t[1]), float(med_t[2]),
            float(med_q[0]), float(med_q[1]),
            float(med_q[2]), float(med_q[3])
        )

        self.create_tf_tree("mir", f"Median_{child}", pose)

    # --------------------------------------------------
    # TF Callback
    # --------------------------------------------------
    def tf_callback(self, msg: TFMessage):
        if not msg.transforms:
            return

        msg_out = TagvectorArray()

        for transform in msg.transforms:
            if transform.header.frame_id != "camera_color_optical_frame":
                continue

            child = transform.child_frame_id
            tag_key = child.split(':')[-1]

            crd = self.get_tf_tree_crd("mir", child)
            if crd is None:
                continue

            tx, ty, tz, rx, ry, rz, rw = crd
            t = self.time_to_sec(transform.header.stamp)

            meas_pos = np.array([tx, ty, tz], dtype=float)
            filt_pos, filt_vel = self.alpha_beta_update(tag_key, meas_pos, t)

            est_pose = (
                float(filt_pos[0]),
                float(filt_pos[1]),
                float(filt_pos[2]),
                rx, ry, rz, rw
            )

            if tag_key not in self.est_tag_pos:
                self.est_tag_pos[tag_key] = deque(maxlen=self.history_size)

            self.est_tag_pos[tag_key].append((t, est_pose))

            self.create_tf_tree("mir", f"est_{child}", est_pose)
            self.median_pose_from_est(tag_key, child)

            speed = float(np.linalg.norm(filt_vel))
            if speed > 1e-9:
                unit = filt_vel / speed
            else:
                unit = np.zeros(3)

            vec = Tagvector()
            vec.header = Header()
            vec.header.stamp = self.get_clock().now().to_msg()
            vec.header.frame_id = "Tagvector"
            vec.tag_id = f"est_{child}"
            vec.tag_id_only_nr = int(tag_key)

            vec.vx, vec.vy, vec.vz = map(float, filt_vel)
            vec.vx_unit, vec.vy_unit, vec.vz_unit = map(float, unit)
            vec.speed = speed

            msg_out.vectors.append(vec)

        self.tagvector_publisher.publish(msg_out)


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
