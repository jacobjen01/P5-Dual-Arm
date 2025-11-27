import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from p5_interfaces.srv import PoseConfig


class PoseConfigListener(Node):

    def __init__(self):
        super().__init__('p5_pose_config_listener')
        self.bob = [0, 0, 0, 0, 0, 0]
        self.alice = [0, 0, 0, 0, 0, 0]
        self.srv = self.create_service(PoseConfig, '/p5_pose_config', self.change_pose)
        self.joint_values = self.create_subscription(JointState, '/joint_states', self.get_joint_val, 10)

    def change_pose(self, request, response):
        if self.bob == [0, 0, 0, 0, 0, 0] or self.alice == [0, 0, 0, 0, 0, 0]:
            response.message = False
            return response
        try:
            with open("config/pre_config_poses.json", "r") as f:
                data = f.read()
            data = json.loads(data)
        except:
            data = {}

        if request.robot_name == 'BOB' or request.robot_name == 'bob':
            data[request.pose] = self.bob
        else:
            data[request.pose] = self.alice

        json_str = json.dumps(data, indent=4)
        with open("config/pre_config_poses.json", "w") as f:
            f.write(json_str)

        response.message = True

        return response

    def get_joint_val(self, msg):
        self.alice[0] = msg.position[2]
        self.alice[1] = msg.position[1]
        self.alice[2] = msg.position[0]
        self.alice[3] = msg.position[3]
        self.alice[4] = msg.position[4]
        self.alice[5] = msg.position[5]
        self.bob[0] = msg.position[8]
        self.bob[1] = msg.position[7]
        self.bob[2] = msg.position[6]
        self.bob[3] = msg.position[9]
        self.bob[4] = msg.position[10]
        self.bob[5] = msg.position[11]


def main(args=None):
    rclpy.init(args=args)

    node = PoseConfigListener()

    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
