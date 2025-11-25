import rclpy
import numpy as np
import json
import time
import threading

from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from p5_safety._error_handling import ErrorHandler

from p5_interfaces.srv import SaveProgram


class ProgramLoader(Node):
    def __init__(self):
        super().__init__('program_loader')
        self.JSON_PATH = ""

        self.error_handler = ErrorHandler()

        self.save_program_service = self.create_service(SaveProgram, f'program_executor/load_program',
                                                        self.save_program_callback)

    def save_program_callback(self, req, resp):
        try:
            with open(self.JSON_PATH, 'r') as f:
                json_data = json.loads(f.read())

            json_data[req.program_name] = req.json_data

            with open(self.JSON_PATH, 'w') as f:
                f.write(json.dumps(json_data))

            resp.resp = True
            return resp
        except Exception as e:
            self.get_logger().error(f'Error while saving program: {e}')
            self.error_handler.report_error(self.error_handler.warning,
                                            f'Failed to save program: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ProgramLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
