import time
import json

import rclpy
import math
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from p5_interfaces.srv import RobotConfigurations


class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager_client')
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /controller_manager/switch_controller...')

    def switch_controller(self, controllers_to_stop, controllers_to_start):
        request = SwitchController.Request()
        request.deactivate_controllers = controllers_to_stop
        request.activate_controllers = controllers_to_start
        request.strictness = 2  # Use STRICT
        request.activate_asap = True  # Activate the new controller as soon as possible

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f'Successfully switched controllers: {controllers_to_stop} -> {controllers_to_start}')
        else:
            self.get_logger().error('Failed to call service: %r' % future.exception())


class JTCClient(rclpy.node.Node):
    """Small test client for the jtc."""

    def __init__(self):
        super().__init__("jtc_client")
        self.home_service = self.create_service(
            RobotConfigurations,
            "/robot_configurations",
            self.handle_robot_configuration_service)

    def handle_robot_configuration_service(self, request, response):
        self.handle_controller(request.robot_name)
        self.parse_trajectories(request.goal_name)
        self.execute_trajectory()
        response.success = True
        response.message = "HOME"
        return response

    def handle_controller(self, robot_name):
        controller_name = robot_name + "_scaled_joint_trajectory_controller/follow_joint_trajectory"
        self.joints = [
            robot_name + "_shoulder_pan_joint",
            robot_name + "_shoulder_lift_joint",
            robot_name + "_elbow_joint",
            robot_name + "_wrist_1_joint",
            robot_name + "_wrist_2_joint",
            robot_name + "_wrist_3_joint"
        ]
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()
        self._send_goal_future = None
        self._get_result_future = None

    def parse_trajectories(self, goal_name):
        with open("pre_config_poses.json", "r") as f:
            TRAJECTORIES = f.read()
        TRAJECTORIES = json.loads(TRAJECTORIES)

        self.goal_pose = JointTrajectory()
        self.goal_pose.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = TRAJECTORIES[goal_name]
        point.velocities = [0, 0, 0, 0, 0, 0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        self.goal_pose.points.append(point)

    def execute_trajectory(self):
        self.get_logger().info(f"Executing trajectory")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goal_pose

        goal.goal_time_tolerance = Duration(sec=0, nanosec=10)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Done with result: {self.error_code_to_str(result.error_code)}"
            )
        raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def switch_to_joint_control():
    controller_manager = ControllerManager()

    controllers_to_stop = ['alice_forward_position_controller']
    controllers_to_start = ['alice_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop, controllers_to_start)

    controllers_to_stop = ['bob_forward_position_controller']
    controllers_to_start = ['bob_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop, controllers_to_start)

    controller_manager.destroy_node()


def switch_to_position_control():
    controller_manager = ControllerManager()

    controllers_to_start = ['alice_forward_position_controller']
    controllers_to_stop = ['alice_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop, controllers_to_start)

    controllers_to_start = ['bob_forward_position_controller']
    controllers_to_stop = ['bob_scaled_joint_trajectory_controller']
    controller_manager.switch_controller(controllers_to_stop, controllers_to_start)

    controller_manager.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    client = JTCClient()
    switch_to_joint_control()
    try:
        rclpy.spin(client)
    except RuntimeError as err:
        client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger().info("Done")
    switch_to_position_control()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
