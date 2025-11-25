import json

import rclpy
from rclpy.node import Node

from p5_interfaces.srv import PoseConfig


class PoseConfigListener(Node):

    def __init__(self):
        super().__init__('p5_pose_config_listener')
        self.srv = self.create_service(PoseConfig, '/p5_pose_config', self.change_pose)

    def change_pose(self, request, response):
        try:
            with open("config/pre_config_poses.json", "r") as f:
                data = f.read()
            data = json.loads(data)
        except:
            data = {}
        data[request.pose] = request.joints
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
