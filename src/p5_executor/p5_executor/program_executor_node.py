import rclpy
import json
import time
import threading
import asyncio

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from ur_msgs.srv import SetIO
from p5_safety._error_handling import ErrorHandler
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import SwitchController

from p5_interfaces.srv import LoadProgram, RunProgram, MoveToPose, MoveToPreDefPose, GetStatus
from p5_interfaces.srv import AdmittanceSendData, AdmittanceSetStatus, LoadAdmittanceParam
from p5_interfaces.msg import CommandState


class ProgramExecutor(Node):
    def __init__(self):
        super().__init__('program_executor')

        self.JSON_PATH = "config/programs.json"
        self.SLEEP_TIME = 0.005
        self.LOOKUP = {
            "c_move": self._command_c_move,
            "r_move": self._command_r_move,
            "r_move_fe": self._command_r_move_fe,
            "sync": self._command_synchronize,
            "frame_availability": self._command_frame_availability,
            "grip": self._command_grip,
            "admittance": self._command_admittance,
            "delay": self._command_delay,
        }

        self.error_handler = ErrorHandler(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.load_program_service = self.create_service(LoadProgram, 'program_executor/load_program',
                                                        self.load_program_callback)

        self.load_program_json_service = self.create_service(LoadProgram, 'program_executor/load_raw_JSON',
                                                        self.load_raw_json_callback)
        
        self.run_program_service = self.create_service(RunProgram, 'program_executor/run_program',
                                                       self.run_program_callback)
        
        self.command_state_subscriber = self.create_subscription(CommandState,
                                                            "p5_command_state",
                                                            self.get_command_state, 10)

        self.cache = {} # Stores temporary values which multiple threads require access to.
        self.feedback = {}
        self.threads = []

        self.program = None

    def get_command_state(self, msg):
        self.feedback[f'{msg.robot_name}_{msg.cmd}'] = msg.status

    def load_program_callback(self, request, response):
        program_name = request.json_data

        with open(self.JSON_PATH, "r") as f:
            programs = json.loads(f.read())

        if program_name not in programs:
            response.success = False
            return response

        self.program = programs[program_name]

        response.success = True
        return response

    def load_raw_json_callback(self, request, response):
        raw_json = request.json_data
        programs = json.loads(raw_json)
        keys = list(programs.keys())
        self.program = programs[keys[0]]
        self.get_logger().info(f'{programs}')

        response.success = True
        return response

    def run_program_callback(self, request, response):
        if self.program is None:
            response.resp = False
            return response

        self.threads = []
        if request.status:
            for thread_data in self.program['threads']:
                self.threads.append(threading.Thread(target=self._run_program_thread, args=(thread_data,)))

            for thread in self.threads:
                thread.start()

        else:
            pass  # evt. kode for at håndtere stop af program.
        response.resp = True
        return response

    def switch_controller(self, controller_activate, controller_deactivate):
        client, req = self._get_client_and_request(SwitchController, '/controller_manager/switch_controller')

        req.deactivate_controllers = [controller_deactivate]
        req.activate_controllers = [controller_activate]
        req.strictness = 2  # Use STRICT
        req.activate_asap = True  # Activate the new controller as soon as possible

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)

    def _run_program_thread(self, thread_data):
        name = thread_data['name']
        robot_name = thread_data['robot_name']
        commands = thread_data['commands']

        for command in commands:
            command_name = command['command']
            args = command['args']
            if command_name in self.LOOKUP:
                self.get_logger().info(f'Robot {robot_name}, command {command_name}, args {args}')
                self.LOOKUP[command_name](name, robot_name, args) # Runs a function defined in self.LOOKUP.

    def _command_c_move(self, name, robot_name, args):

        config_name = args['config_name']
        self.switch_controller(f'{robot_name}_scaled_joint_trajectory_controller', f'{robot_name}_forward_position_controller')
        client, req = self._get_client_and_request(MoveToPreDefPose, 'p5_move_to_pre_def_pose')

        req.robot_name = robot_name
        req.goal_name = config_name

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)
        while not f'{robot_name}_c_move' in self.feedback:
            continue
        while not self.feedback[f'{robot_name}_c_move']:
            continue

    def _command_r_move(self, name, robot_name, args):
        pose = args['pose']
        linear = args['linear']
        use_tracking_velocity = args['use_tracking_velocity']
        frame = args['frame']
        self.switch_controller(f'{robot_name}_forward_position_controller', f'{robot_name}_scaled_joint_trajectory_controller')

        servo_node_command_client = self.create_client(ServoCommandType, f'{robot_name}/servo_node/switch_command_type')

        while not servo_node_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        cmd_type = ServoCommandType.Request()
        cmd_type.command_type = 2

        future = servo_node_command_client.call_async(cmd_type)
        self.executor.spin_until_future_complete(future)

        client, req = self._get_client_and_request(MoveToPose, f"{robot_name}/p5_move_to_pose")

        req.pose.position.x = pose[0]
        req.pose.position.y = pose[1]
        req.pose.position.z = pose[2]
        req.pose.orientation.x = pose[3]
        req.pose.orientation.y = pose[4]
        req.pose.orientation.z = pose[5]
        req.pose.orientation.w = pose[6]

        req.linear = linear
        req.use_tracking_velocity = use_tracking_velocity
        req.frame = frame

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)
        time.sleep(0.2)
        client, req = self._get_client_and_request(GetStatus, f"{robot_name}/p5_relative_mover_status")
        while True:
            future = client.call_async(req)
            self.executor.spin_until_future_complete(future)
            time.sleep(0.2)
            response = future.result()
            if response.running:
                break

    def _command_r_move_fe(self, name, robot_name, args):
        goal_foce = args['force']
        pose = args['pose']
        linear = args['linear']
        use_tracking_velocity = args['use_tracking_velocity']
        frame = args['frame']
        self.switch_controller(f'{robot_name}_forward_position_controller', f'{robot_name}_scaled_joint_trajectory_controller')

        servo_node_command_client = self.create_client(ServoCommandType, f'{robot_name}/servo_node/switch_command_type')

        while not servo_node_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        cmd_type = ServoCommandType.Request()
        cmd_type.command_type = 2

        future = servo_node_command_client.call_async(cmd_type)
        self.executor.spin_until_future_complete(future)

        client, req = self._get_client_and_request(MoveToPose, f"{robot_name}/p5_move_to_pose")

        req.pose.position.x = pose[0]
        req.pose.position.y = pose[1]
        req.pose.position.z = pose[2]
        req.pose.orientation.x = pose[3]
        req.pose.orientation.y = pose[4]
        req.pose.orientation.z = pose[5]
        req.pose.orientation.w = pose[6]

        req.linear = linear
        req.use_tracking_velocity = use_tracking_velocity
        req.frame = frame

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)
        time.sleep(0.2)
        client, req = self._get_client_and_request(AdmittanceSendData, f"{robot_name}/p5_admittance_get_force_torque")
        while True:
            future = client.call_async(req)
            self.executor.spin_until_future_complete(future)
            response = future.result()
            time.sleep(1)
            self.get_logger().info(f'{abs(response.ft[0])}, {abs(response.ft[1])}, {abs(response.ft[2])}')
            if goal_foce[0] < abs(response.ft[0]) and goal_foce[1] < abs(response.ft[1]) and goal_foce[2] < abs(response.ft[2]):
                break
        self.get_logger().info('force reached')
        self.switch_controller(f'{robot_name}_scaled_joint_trajectory_controller', f'{robot_name}_forward_position_controller')

    def _command_synchronize(self, name, robot_name, args):
        cache_id = f"sync_{args["sync_id"]}"

        if cache_id in self.cache:
            self.cache[cache_id].append(name)
        else:
            self.cache[cache_id] = [name]
        while not len(self.cache[cache_id]) == len(args['threads']):
            continue

    def _command_frame_availability(self, name, robot_name, args):
        frame = args['frame_name']
        try:
            frames = self.tf_buffer.all_frames_as_string()

            while not frame in frames:
                continue

        except Exception as e:
            self.get_logger().warn(f"Could not get frames: {e}")
            self.error_handler.report_error(self.error_handler.info,
                                            f'Could not get frames: {e}')
            self._command_frame_availability(name, robot_name, args)

    def _command_grip(self, name, robot_name, args):
        state = args['action']

        client, req = self._get_client_and_request(SetIO, f'/{robot_name}_io_and_status_controller/set_io')

        req.fun = 1
        req.pin = 16
        req.state = 0.0
        if state == 'close':
            req.state = 1.0

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)

        client, req = self._get_client_and_request(SetIO, f'/{robot_name}_io_and_status_controller/set_io')

        req.fun = 1
        req.pin = 17
        req.state = 0.0
        if state == 'open':
            req.state = 1.0

        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)

    def _command_admittance(self, name, robot_name, args):
        param_name = args['parameters_name']
        action = [args['fx'], args['fy'], args['fz'], args['tx'], args['ty'], args['tz']]

        client, req = self._get_client_and_request(LoadAdmittanceParam, f'{robot_name}/p5_load_admittance_param')
        req.param_name = param_name
        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)

        client, req = self._get_client_and_request(AdmittanceSetStatus, f'{robot_name}/p5_admittance_set_state')
        req.active = action
        future = client.call_async(req)
        self.executor.spin_until_future_complete(future)

    def _command_delay(self, name, robot_name, args):
        delay_time = args['time']
        end_time = time.time() + delay_time
        self.get_logger().info('timer started')
        while time.time() <= end_time:
            continue
        self.get_logger().info('timer ended')

    def _get_client_and_request(self, datatype, service):
        client = self.create_client(datatype, service)

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        req = datatype.Request()

        return client, req


def main(args=None):
    rclpy.init(args=args)
    node = ProgramExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    #executor = rclpy.executors.MultiThreadedExecutor()
    #executor.add_node(node)
    #try:
    #    executor.spin()
    #finally:
    #    node.destroy_node()
    #    rclpy.shutdown()


if __name__ == '__main__':
    main()
