import re
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from collections import deque
import math
from p5_interfaces.msg import TagvectorArray, Tagvector
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
from p5_safety._error_handling import ErrorHandler
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Header
import time

class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.error_handler = ErrorHandler(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laver parametere (kan overskrives via ros2 param eller launch)
        self.declare_parameter('input_topic', '/tf')                                                                    # Navn på topic vi subber fra
        self.declare_parameter('output_topic', '/future_tag_vector')                                                    # Navn på topic vi publicerer til
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 10)                                                                       # Antal positioner vi gemmer i historikken pr tag til udregning af bevægelse
        self.declare_parameter('use_averaging', True)
        self.declare_parameter('median_window', 5)   # window size for median filter (use 1 to disable)
 
        # Henter parameterværdier
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value                              # Henter input topic navn fra parameter
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value                            # Henter output topic navn fra parameter
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value                                 # Henter qos depth fra parameter
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value                      # Henter history size fra parameter
        self.use_averaging = self.get_parameter('use_averaging').get_parameter_value().bool_value                       # Henter use_averaging fra parameter
        self.median_window = int(self.get_parameter('median_window').get_parameter_value().integer_value)

        # Opretter QoS profil
        qos = QoSProfile(depth=qos_depth)

        # Opretter publisher til output topic. Sætter beskedtypen til vores custom type Tagvector
        self.tagvector_publisher = self.create_publisher(TagvectorArray, output_topic, qos)

        # Opretter subscription til input topic. Her er det /tf fra apriltag detektering
        self.subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self.tf_callback,
            qos
        )

        self.tag_poses = {}      # Liste til at gemme nuværende poses { tag_key: {'translation': (x,y,z), 'rotation': (qx,qy,qz,qw)} }
        self.tag_history = {}    # Liste til at gemme historik { tag_key: deque([ (t, (x,y,z)), ... ], maxlen=history_size) }
        self.tag_motion = {}     # Liste til at gemme bevægelse { tag_key: {'direction': (dx,dy,dz), 'speed': s} }
        self.vector_history = {} # Liste til at gemme vektor historik { tag_key: deque([ (vx, vy, vz), ... ], maxlen=history_size) }
        self.est_tag_pos = {}    # Liste til at gemme estimerede fremtidige positioner { tag_key: deque([ (vx, vy, vz), ... ], maxlen=history_size) }

        #self.get_logger().info(f'Subscribed to: {input_topic} -> Publishing to: {output_topic}')

    def median_pose_from_est(self, tag_key, child):
        # Apply median over estimated positions (post-smoothing)
        if self.median_window > 1 and tag_key in self.est_tag_pos and len(self.est_tag_pos[tag_key]) > 0:
            est_hist = list(self.est_tag_pos[tag_key])
            window = min(self.median_window, len(est_hist))
            recent = np.array([p[1] for p in est_hist[-window:]], dtype=float)  # shape (n,7)
            if recent.shape[0] > 0:
                med_t = np.median(recent[:, 0:3], axis=0)
                med_q = np.median(recent[:, 3:7], axis=0)
                norm = np.linalg.norm(med_q)
                if norm > 1e-12:
                    med_q = med_q / norm
                else:
                    med_q = recent[-1, 3:7]
                mtx, mty, mtz = float(med_t[0]), float(med_t[1]), float(med_t[2])
                mrx, mry, mrz, mrw = float(med_q[0]), float(med_q[1]), float(med_q[2]), float(med_q[3])
                self.create_tf_tree("mir", f"Median_{child}", (mtx, mty, mtz, mrx, mry, mrz, mrw))

    def create_tf_tree(self, parent_name, child_name, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_name
        t.child_frame_id = child_name

        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]

        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]

        self.tf_broadcaster.sendTransform(t)

    def get_tf_tree_crd(self, parent_name, child_name):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(parent_name, child_name, now)

            crd = np.array([trans.transform.translation.x, trans.transform.translation.y,
                            trans.transform.translation.z, trans.transform.rotation.x,
                            trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])

            return crd

        except Exception as e:
            self.get_logger().error(f'Failed to get transform from {parent_name}/{child_name}: {e}')
            self.error_handler.report_error(self.error_handler.info, f'Failed to get transform from {parent_name}/{child_name}: {e}')

            return []

    def time_to_sec(self, stamp):
        # Timestampen kommer fra builtin_interfaces.msg.Time som har to fields: sec og nanosec
        # Derfor skal vi kombinere dem til en enkelt float værdi i sekunder.
        return float(stamp.sec) + float(getattr(stamp, 'nanosec', 0)) * 1e-9
    
    def compute_motion_from_history(self, history_deque):
        # History_deque er en deque der indeholder tuples (t, (x,y,z)), ældste til nyeste
        if len(history_deque) < 2:                              # Håndterer tilfælde med for få data
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        
        t_old, pos_old = history_deque[0]                       # Sætter ældste tid og position
        if len(history_deque) < 5:
            t_new, pos_new = history_deque[-1]                  # Sætter nyeste tid og position
        else:
            t_new, pos_new = history_deque[4]                      # sætter nyeste tid og position
        dt = t_new - t_old                                      # Tidsforskel

        if dt <= 0:                                             # Håndterer tilfælde med ugyldig tidsforskel
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        
        vx = (pos_new[0] - pos_old[0]) / dt                     # Beregning af x hastighed komponent
        vy = (pos_new[1] - pos_old[1]) / dt                     # Beregning af y hastighed komponent
        vz = (pos_new[2] - pos_old[2]) / dt                     # Beregning af z hastighed komponent
        if not self.use_averaging:
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)                # Beregning af samlet hastighed

            if speed == 0.0:                                        # Håndterer tilfælde med ingen bevægelse
                direction_unit = (0.0, 0.0, 0.0)
                direction = (0.0, 0.0, 0.0)
            else:                                                   # Normaliserer retningen og sætter hastighedsvektorer
                direction_unit = (vx/speed, vy/speed, vz/speed)
                direction = (vx, vy, vz)

            return direction_unit, direction, speed

        else:
            direction = (vx, vy, vz)
            return direction, direction, 0.0  # Speed will be computed later in averaging

    def average_motion(self, vector_deque):
        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0

        if len(vector_deque) == 0:                                                          # Håndterer tilfælde med ingen data
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0

        for vx, vy, vz in vector_deque:                                                     # Summerer alle vektorer for hvert komponent
            sum_x += vx
            sum_y += vy
            sum_z += vz
        
        avg_vx = sum_x / len(vector_deque)                                                  # Beregner gennemsnit for hvert komponent
        avg_vy = sum_y / len(vector_deque)
        avg_vz = sum_z / len(vector_deque)

        avg_direction = (avg_vx, avg_vy, avg_vz)                                            # Samlet gennemsnitsretning

        avg_speed = math.sqrt(avg_vx*avg_vx + avg_vy*avg_vy + avg_vz*avg_vz)                # Beregner samlet gennemsnitshastighed
        if avg_speed == 0.0:
            avg_direction_unit = (0.0, 0.0, 0.0)
            avg_direction = (0.0, 0.0, 0.0)
        else:
            avg_direction_unit = (avg_vx/avg_speed, avg_vy/avg_speed, avg_vz/avg_speed)     # Normaliserer gennemsnitsretningen

        return avg_direction, avg_direction_unit, avg_speed

    def calculate_next_position(self, new_point, point_deque, direction, time):
        if len(point_deque) == 0:
            return new_point
        
        dt = time - point_deque[-1][0]  # Tidsforskel mellem nyeste punkt i deque og det nye punkt
        if dt <= 0:
            return new_point
        
        dx = direction[0] * dt
        dy = direction[1] * dt
        dz = direction[2] * dt
        next_x = point_deque[-1][1][0] + dx
        next_y = point_deque[-1][1][1] + dy
        next_z = point_deque[-1][1][2] + dz

        # Average out position of new point and predicted next point
        avg_x = (new_point[0] + next_x) / 2.0
        avg_y = (new_point[1] + next_y) / 2.0
        avg_z = (new_point[2] + next_z) / 2.0

        return (avg_x, avg_y, avg_z, new_point[3], new_point[4], new_point[5], new_point[6])

    def tf_callback(self, msg: TFMessage):
        start_time = time.perf_counter()

        if not msg.transforms:                                              # Hånderer tilfælde med tom TFMessage
            #self.get_logger().warning("Received empty TFMessage")
            #self.tag_history.clear()
            return

        #Check at der er en transform der er et tag
        has_tag_frame = any(transform.header.frame_id == "camera_color_optical_frame" for transform in msg.transforms)
        if has_tag_frame:
            msg_out_array = TagvectorArray()

            for transform in msg.transforms:                                    # Går igennem alle transformer dvs alle tags i beskeden
                if transform.header.frame_id != "camera_color_optical_frame":   # Vi er kun interesseret i tags der er i world frame
                    continue
                child = transform.child_frame_id                                # Henter child frame id (tag navn)
                if ':' in child:                                                # Tjekker om der er et kolon i navnet
                    tag_key = child.split(':')[-1]                              # Snupper delen efter kolonet som tag_key da apriltag bruger formatet tag36h11:X
                else:
                    tag_key = child.replace('/', '_')                           # Håndterer andre frame navne

                crd = self.get_tf_tree_crd("mir", child)            # Opdaterer transform for tag i forhold til alice_base_link
                if len(crd) == 0:
                    continue
                tx, ty, tz, rx, ry, rz, rw = crd

                # Får timestamp i sekunder
                stamp = transform.header.stamp
                t=self.time_to_sec(stamp)

                # Sikrer at der er en deque til historik for dette tag
                if tag_key not in self.tag_history:
                    self.tag_history[tag_key] = deque(maxlen=self.history_size)
                self.tag_history[tag_key].append((t, (tx, ty, tz, rx, ry, rz, rw)))  # Tilføjer ny position til historikken

                # --- APPLY MEDIAN FILTER OVER LAST self.median_window MEASUREMENTS ---
                # If median_window == 1 this is effectively disabled (keeps current measurement)
                # Disabled: median now applied after smoothing on estimated poses
                if False and self.median_window > 1:
                    pass

                # Sikrer at der er en deque til vektor historik for dette tag
                if tag_key not in self.vector_history:
                    self.vector_history[tag_key] = deque(maxlen=self.history_size)

                if tag_key not in self.est_tag_pos:
                    self.est_tag_pos[tag_key] = deque(maxlen=self.history_size)

                # Udregn bevægelse ud fra historikken
                if len(self.est_tag_pos[tag_key]) == 0:
                    direction_unit, direction, speed = self.compute_motion_from_history(self.tag_history[tag_key])
                else:
                    direction_unit, direction, speed = self.compute_motion_from_history(self.est_tag_pos[tag_key])
                self.tag_motion[tag_key] = {'direction_unit': direction_unit, 'direction': direction, 'speed': speed}

                # Logger bevægelsesinfo
                # self.get_logger().info(f"Tag {tag_key} motion -> dir: ({direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}), speed: {speed:.3f} m/s")

                # Decide whether to average
                if self.use_averaging:
                    # store latest instantaneous direction (vx,vy,vz)
                    self.vector_history[tag_key].append(direction)
                    #self.get_logger().info(f"Tag {tag_key} vector history length: {len(self.vector_history[tag_key])}")
                    avg_direction, avg_direction_unit, avg_speed = self.average_motion(self.vector_history[tag_key])
                    vx_used, vy_used, vz_used = avg_direction
                    vx_unit_used, vy_unit_used, vz_unit_used = avg_direction_unit
                    speed_used = avg_speed

                    est_point = self.calculate_next_position(
                        (tx, ty, tz, rx, ry, rz, rw),
                        self.est_tag_pos[tag_key],
                        avg_direction,
                        t
                    )
                    self.est_tag_pos[tag_key].append((t, est_point))
                    self.create_tf_tree("mir", f"est_{child}", est_point)
                    # Apply median after smoothing on estimated poses
                    self.median_pose_from_est(tag_key, child)

                else:
                    vx_used, vy_used, vz_used = direction
                    vx_unit_used, vy_unit_used, vz_unit_used = direction_unit
                    speed_used = speed
                    # Keep estimated pose consistent by using current measurement
                    est_point = (tx, ty, tz, rx, ry, rz, rw)
                    self.est_tag_pos[tag_key].append((t, est_point))
                    self.create_tf_tree("mir", f"est_{child}", est_point)
                    # Apply median after smoothing on estimated poses
                    self.median_pose_from_est(tag_key, child)

                # Opretter Tagvector besked for dette tag
                vector = Tagvector()
                vector.header = Header()
                vector.header.stamp = self.get_clock().now().to_msg()
                vector.header.frame_id = "Tagvector"
                vector.tag_id = f'est_{child}'
                vector.tag_id_only_nr=int(tag_key)
                vector.vx = vx_used
                vector.vy = vy_used
                vector.vz = vz_used
                vector.vx_unit = vx_unit_used
                vector.vy_unit = vy_unit_used
                vector.vz_unit = vz_unit_used
                vector.speed = speed_used

                msg_out_array.vectors.append(vector)
            
            # Publiserer TagvectorArray beskeden
            # self.get_logger().info(f"Publishing future tag vectors for {len(msg_out_array.vectors)} tags")
            self.tagvector_publisher.publish(msg_out_array)

            # Always log how long this callback took
        duration = time.perf_counter() - start_time
        # self.get_logger().info(f"tf_callback duration: {duration:.6f} s")

            # self.get_logger().info(f"Current tag poses: {self.tag_poses}")

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
