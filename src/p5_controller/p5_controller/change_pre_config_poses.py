import json

import rclpy
from rclpy.node import Node

from p5_interfaces.srv import PoseConfig 

class PoseConfigListener(Node):

    def __init__(self):
        super().__init__('pose_config_listener')
        self.srv = self.create_service(PoseConfig, '/pose_config', self.change_pose)

    def change_pose(self, request, response):
        try:
            with open("pre_config_poses.json", "r") as f:
                data = f.read()
            data = json.loads(data)
        except:
            data = {}
        data[request.pose] = [
                request.joint_1,
                request.joint_2,
                request.joint_3,
                request.joint_4,
                request.joint_5,
                request.joint_6
        ]
        json_str = json.dumps(data, indent=4)
        with open("pre_config_poses.json", "w") as f:
            f.write(json_str)

        response.message = "writen to json file"

        return response


def main(args=None):
    rclpy.init(args=args)

    node = PoseConfigListener()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
