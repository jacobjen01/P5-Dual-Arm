import rclpy
from rclpy.node import Node
import requests

class MirRestNode(Node):
    headers = {
        "accept": "application/json",
        "Authorization": "Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA==",
        "Accept-Language": "en_US"
    }
    def __init__(self):
        super().__init__('mir_rest_node')

        self.timer = self.create_timer(3.0, self.get_system_info)

    def get_system_info(self):
        ## Retrieve the information about the system. It contains different information like serial numbers of hardware components, MAC addresses of network cards, etc…
        url = "http://192.168.12.20/api/v2.0.0/system/info"

        try:
            response = requests.get(url, headers=self.headers, timeout=2)
            print("Status:", response.status_code)
            print("Text:", response.text)
            if response.ok:
                try:
                    print(response.json())
                except Exception as e:
                    print("JSON parse error:", e)
            else:
                print("Error:", response.status_code, response.text)
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MirRestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()