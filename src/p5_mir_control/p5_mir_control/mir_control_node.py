#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
from urllib.parse import urljoin
from p5_interfaces.srv import GetMissions, PostMissions


class MirRestNode(Node):
    headers = {
        "accept": "application/json",
        "Authorization": "Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA==",
        "Accept-Language": "en_US",
        "Content-Type": "application/json"
    }

<<<<<<< HEAD
    BASE_URL = "http://192.168.12.20/api/v2.0.0/"

=======
>>>>>>> cb50e4f02f67e817fbc09cf9e1259343a4609a4d
    def __init__(self):
        super().__init__('mir_rest_node')
        # services
        self.get_missions_srv = self.create_service(GetMissions, 'get_missions', self.handle_get_missions)
        self.post_mission_srv = self.create_service(PostMissions, 'post_mission', self.handle_post_mission)
        self.get_logger().info("mir_rest_node ready: services -> /get_missions, /post_mission")

<<<<<<< HEAD
        # Run interactive mission selection automatically
        self.select_and_post_mission()
=======
        self.timer = self.create_timer(3.0, self.get_system_info)

    def get_system_info(self):
        # Retrieve the information about the system. It contains different information like serial numbers of hardware components, MAC addresses of network cards, etc…
        url = "http://192.168.12.20/api/v2.0.0/system/info"
>>>>>>> cb50e4f02f67e817fbc09cf9e1259343a4609a4d

    def handle_get_missions(self, request, response):
        url = urljoin(self.BASE_URL, 'missions')
        try:
            r = requests.get(url, headers=self.headers, timeout=3)
            if not r.ok:
                response.success = False
                response.message = f"HTTP {r.status_code}: {r.text}"
                return response
            data = r.json()
            guids = []
            names = []
            for item in data:
                guids.append(item.get('guid', ''))
                names.append(item.get('name', ''))
            response.guids = guids
            response.names = names
            response.success = True
            response.message = f"Found {len(guids)} missions"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Exception: {e}"
            return response

    def handle_post_mission(self, request, response):
        guid = request.mission_guid.strip()
        if not guid:
            response.success = False
            response.message = "Empty mission_guid"
            return response

        url = urljoin(self.BASE_URL, 'mission_queue')
        payload = {"mission_id": guid}
        try:
            r = requests.post(url, json=payload, headers=self.headers, timeout=5)
            if r.ok or r.status_code in (200,201):
                response.success = True
                response.message = f"Enqueued mission {guid} (status {r.status_code})"
            else:
                response.success = False
                response.message = f"HTTP {r.status_code}: {r.text}"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Exception: {e}"
            return response

    def select_and_post_mission(self):
        self.get_logger().info("Fetching missions from MiR...")
        url = urljoin(self.BASE_URL, 'missions')
        try:
            r = requests.get(url, headers=self.headers, timeout=3)
            r.raise_for_status()
            data = r.json()
        except Exception as e:
            self.get_logger().error(f"Failed to fetch missions: {e}")
            return

        # Filter only ChargeMir and MoveLinear
        target_missions = {item['name']: item['guid'] for item in data if item['name'] in ['ChargeMir', 'MoveLinear']}
        if not target_missions:
            self.get_logger().warning("No target missions (ChargeMir, MoveLinear,) found!")
            return

        print("\nAvailable missions to queue:")
        for name in target_missions.keys():
            print(f"- {name}")

        # choose by name (case-insensitive)
        choice = input("Enter mission name to queue (case-insensitive): ").strip()
        if not choice:
            self.get_logger().error("Empty selection!")
            return

        # Support case-insensitive matching
        name_map = {n.lower(): n for n in target_missions.keys()}
        selected_key = name_map.get(choice.lower())
        if not selected_key:
            self.get_logger().error(f"Invalid selection: '{choice}'")
            return

        selected_name = selected_key
        selected_guid = target_missions[selected_name]

        client = self.create_client(PostMissions, 'post_mission')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /post_mission service...')
        req = PostMissions.Request()
        req.mission_guid = selected_guid
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Mission '{selected_name}' queued successfully!")
        else:
            msg = future.result().message if future.result() else "No response"
            self.get_logger().error(f"Failed to queue '{selected_name}': {msg}")



def main(args=None):
    rclpy.init(args=args)
    node = MirRestNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
