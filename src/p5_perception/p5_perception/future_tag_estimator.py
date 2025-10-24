import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from collections import deque
import math
from p5_interfaces.msg import Tagvector
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer
import numpy as np

class FutureTagEstimator(Node):
    def __init__(self):
        super().__init__('future_tag_estimator')

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Laver parametere (kan overskrives via ros2 param eller launch)
        self.declare_parameter('input_topic', '/tf')                                                                    # Navn på topic vi subber fra
        self.declare_parameter('output_topic', '/future_tag_vector')                                                    # Navn på topic vi publicerer til
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('history_size', 5)                                                                       # Antal positioner vi gemmer i historikken pr tag til udregning af bevægelse

        # Henter parameterværdier
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value                              # Henter input topic navn fra parameter
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value                            # Henter output topic navn fra parameter
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value                                 # Henter qos depth fra parameter
        self.history_size = self.get_parameter('history_size').get_parameter_value().integer_value                      # Henter history size fra parameter

        # Opretter QoS profil
        qos = QoSProfile(depth=qos_depth)

        # Opretter publisher til output topic. Sætter beskedtypen til vores custom type Tagvector
        self.tagvector_publisher = self.create_publisher(Tagvector, output_topic, qos)

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

        #self.get_logger().info(f'Subscribed to: {input_topic} -> Publishing to: {output_topic}')

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
                            trans.transform.translation.z])

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
        t_new, pos_new = history_deque[-1]                      # sætter nyeste tid og position
        dt = t_new - t_old                                      # Tidsforskel

        if dt <= 0:                                             # Håndterer tilfælde med ugyldig tidsforskel
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        
        vx = (pos_new[0] - pos_old[0]) / dt                     # Beregning af x hastighed komponent
        vy = (pos_new[1] - pos_old[1]) / dt                     # Beregning af y hastighed komponent
        vz = (pos_new[2] - pos_old[2]) / dt                     # Beregning af z hastighed komponent
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)                # Beregning af samlet hastighed

        if speed == 0.0:                                        # Håndterer tilfælde med ingen bevægelse
            direction_unit = (0.0, 0.0, 0.0)
            direction = (0.0, 0.0, 0.0)
        else:                                                   # Normaliserer retningen og sætter hastighedsvektorer
            direction_unit = (vx/speed, vy/speed, vz/speed)
            direction = (vx, vy, vz)

        return direction_unit, direction, speed

    def tf_callback(self, msg: TFMessage):
        if not msg.transforms:                                              # Hånderer tilfælde med tom TFMessage
            self.get_logger().warning("Received empty TFMessage")
            #self.tag_history.clear()
            return

        for transform in msg.transforms:                                    # Går igennem alle transformer dvs alle tags i beskeden
            if transform.header.frame_id != "camera_color_optical_frame":   # Vi er kun interesseret i tags der er i world frame
                continue
            child = transform.child_frame_id                                # Henter child frame id (tag navn)
            if ':' in child:                                                # Tjekker om der er et kolon i navnet
                tag_key = child.split(':')[-1]                              # Snupper delen efter kolonet som tag_key da apriltag bruger formatet tag36h11:X
            else:
                tag_key = child.replace('/', '_')                           # Håndterer andre frame navne

            crd = self.get_tf_tree_crd("alice_base_link", child)            # Opdaterer transform for tag i forhold til alice_base_link
            if len(crd) == 0:
                continue
            tx, ty, tz = crd
            # Tildeler variabler til positions komponenter
            #tx = transform.transform.translation.x
            #ty = transform.transform.translation.y
            #tz = transform.transform.translation.z
            #qx = transform.transform.rotation.x
            #qy = transform.transform.rotation.y
            #qz = transform.transform.rotation.z
            #qw = transform.transform.rotation.w
            #self.tag_poses[tag_key] = {
            #    'translation': (tx, ty, tz),
            #    'rotation': (qx, qy, qz, qw)
            #}

            # Får timestamp i sekunder
            stamp = transform.header.stamp
            t=self.time_to_sec(stamp)

            # Sikrer at der er en deque til historik for dette tag
            if tag_key not in self.tag_history:
                self.tag_history[tag_key] = deque(maxlen=self.history_size)
            self.tag_history[tag_key].append((t, (tx, ty, tz)))


            # Udregn bevægelse ud fra historikken
            direction_unit, direction, speed = self.compute_motion_from_history(self.tag_history[tag_key])
            self.tag_motion[tag_key] = {'direction_unit': direction_unit, 'direction': direction, 'speed': speed}

            # Logger bevægelsesinfo
            self.get_logger().info(f"Tag {tag_key} motion -> dir: ({direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}), speed: {speed:.3f} m/s")

            # self.create_tf_tree(child, "Grabpoint_1", (0.22, 0.0, -0.0815, 0.0, 0.0, 0.0, 1.0))
            # self.create_tf_tree(child, "Grabpoint_2", (-0.22, 0.0, -0.0815, 0.0, 0.0, 0.0, 1.0))

            # Publicerer
            msg_out = Tagvector()
            msg_out.tag_id = int(tag_key)
            msg_out.vx, msg_out.vy, msg_out.vz = direction
            msg_out.vx_unit, msg_out.vy_unit, msg_out.vz_unit = direction_unit
            msg_out.speed = speed
            self.tagvector_publisher.publish(msg_out)

        #for tag_key in self.tag_motion:
        #    msg = Tagvector()
        #    msg.tag_id = int(tag_key)
        #    msg.vx = self.tag_motion[tag_key]['direction'][0]
        #    msg.vy = self.tag_motion[tag_key]['direction'][1]
        #    msg.vz = self.tag_motion[tag_key]['direction'][2]
        #    msg.speed = self.tag_motion[tag_key]['speed']
        #    self.publisher.publish(msg)

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