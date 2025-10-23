import rclpy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FrameMover(Node):
    def __init__(self):
        super().__init__('frame_mover')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.x = 0.0
        self.y = 0.0
        self.rotation_z = 0.0

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "mir"
        t.transform.translation.x = self.x  # Update x position
        t.transform.translation.y = self.y  # Update y position
        t.transform.translation.z = 1.0
        q = quaternion_from_euler(0, 0, self.rotation_z)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # Set rotation if needed
        self.broadcaster.sendTransform(t)
        #self.y += 0.01
        if self.rotation_z >= 360:
            self.rotation_z = 0
        self.rotation_z += 0.01
        self.y += 0.01*math.cos(-self.rotation_z)
        self.x += 0.01*math.sin(-self.rotation_z)

def main(args=None):
    rclpy.init(args=args)
    frame_mover = FrameMover()
    rclpy.spin(frame_mover)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
