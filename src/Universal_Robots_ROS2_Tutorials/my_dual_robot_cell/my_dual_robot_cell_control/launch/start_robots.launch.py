import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup():
    # Initialize Arguments
    alice_ur_type = LaunchConfiguration("alice_ur_type")
    bob_ur_type = LaunchConfiguration("bob_ur_type")

    alice_robot_ip = LaunchConfiguration("alice_robot_ip")
    bob_robot_ip = LaunchConfiguration("bob_robot_ip")

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    headless_mode = LaunchConfiguration("headless_mode")

    # Robot specific arguments
    alice_use_mock_hardware = LaunchConfiguration("alice_use_mock_hardware")
    alice_mock_sensor_commands = LaunchConfiguration("alice_mock_sensor_commands")
    alice_initial_joint_controller = LaunchConfiguration("alice_initial_joint_controller")
    alice_activate_joint_controller = LaunchConfiguration("alice_activate_joint_controller")
    alice_launch_dashboard_client = LaunchConfiguration("alice_launch_dashboard_client")

    bob_use_mock_hardware = LaunchConfiguration("bob_use_mock_hardware")
    bob_mock_sensor_commands = LaunchConfiguration("bob_mock_sensor_commands")
    bob_initial_joint_controller = LaunchConfiguration("bob_initial_joint_controller")
    bob_activate_joint_controller = LaunchConfiguration("bob_activate_joint_controller")
    bob_launch_dashboard_client = LaunchConfiguration("bob_launch_dashboard_client")

    # Single controller manager comprising of controllers for both arms
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
            # We use the tf_prefix as substitution in there, so that's why we keep it as an
            # argument for this launchfile
        ],
        output="screen",
    )

    alice_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(alice_launch_dashboard_client) and UnlessCondition(alice_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="alice",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": alice_robot_ip}],
    )

    bob_dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(bob_launch_dashboard_client) and UnlessCondition(bob_use_mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        namespace="bob",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": bob_robot_ip}],
    )

    alice_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="alice",
        parameters=[{"robot_ip": alice_robot_ip}],
        output="screen",
        condition=UnlessCondition(alice_use_mock_hardware),
    )

    bob_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="bob",
        parameters=[{"robot_ip": bob_robot_ip}],
        output="screen",
        condition=UnlessCondition(bob_use_mock_hardware),
    )

    alice_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="alice",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(alice_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": alice_activate_joint_controller},
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",
                    "alice_io_and_status_controller",
                    "bob_io_and_status_controller",
                    "alice_force_torque_sensor_broadcaster",
                    "bob_force_torque_sensor_broadcaster",
                    "alice_speed_scaling_state_broadcaster",
                    "bob_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    bob_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="bob",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(bob_use_mock_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": bob_activate_joint_controller},
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",
                    "alice_io_and_status_controller",
                    "bob_io_and_status_controller",
                    "alice_force_torque_sensor_broadcaster",
                    "bob_force_torque_sensor_broadcaster",
                    "alice_speed_scaling_state_broadcaster",
                    "bob_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    alice_moveit_config = (
        MoveItConfigsBuilder(robot_name="alice", package_name="my_dual_robot_cell_moveit_config")
        .robot_description_semantic(Path("config") / "my_dual_robot_cell.srdf", {"name": alice_ur_type})
        .to_moveit_configs()
    )

    alice_servo_yaml = load_yaml("my_dual_robot_cell_control", "config/alice_servo.yaml")
    alice_servo_params = {"moveit_servo": alice_servo_yaml}
    alice_servo_node = Node(
        package="moveit_servo",
        namespace="alice",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            alice_moveit_config.to_dict(),
            alice_servo_params,
        ],
        output="screen",
    )

    bob_moveit_config = (
        MoveItConfigsBuilder(robot_name="bob", package_name="my_dual_robot_cell_moveit_config")
        .robot_description_semantic(Path("config") / "my_dual_robot_cell.srdf", {"name": bob_ur_type})
        .to_moveit_configs()
    )

    bob_servo_yaml = load_yaml("my_dual_robot_cell_control", "config/bob_servo.yaml")
    bob_servo_params = {"moveit_servo": bob_servo_yaml}
    bob_servo_node = Node(
        package="moveit_servo",
        namespace="bob",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            bob_moveit_config.to_dict(),
            bob_servo_params,
        ],
        output="screen",
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "alice_io_and_status_controller",
        "bob_io_and_status_controller",
        "alice_speed_scaling_state_broadcaster",
        "bob_speed_scaling_state_broadcaster",
        "alice_force_torque_sensor_broadcaster",
        "bob_force_torque_sensor_broadcaster",
    ]
    controllers_inactive = [
        "alice_forward_position_controller",
        "bob_forward_position_controller",
    ]

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    # There may be other controllers of the joints, but this is the initially-started one
    alice_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            alice_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(alice_activate_joint_controller),
    )
    bob_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            bob_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(bob_activate_joint_controller),
    )
    alice_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            alice_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(alice_activate_joint_controller),
    )
    bob_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            bob_initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(bob_activate_joint_controller),
    )

    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "alice_robot_ip": alice_robot_ip,
            "bob_robot_ip": bob_robot_ip,
            "alice_ur_type": alice_ur_type,
            "bob_ur_type": bob_ur_type,
        }.items(),
    )

    nodes_to_start = [
        control_node,
        alice_dashboard_client_node,
        bob_dashboard_client_node,
        alice_controller_stopper_node,
        bob_controller_stopper_node,
        alice_urscript_interface,
        bob_urscript_interface,
        rsp,
        rviz_node,
       # alice_servo_node,
        bob_servo_node,
        alice_initial_joint_controller_spawner_stopped,
        bob_initial_joint_controller_spawner_stopped,
        alice_initial_joint_controller_spawner_started,
        bob_initial_joint_controller_spawner_started,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_robot_ip",
            default_value="192.168.56.101",
            description="IP address by which alice can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_robot_ip",
            default_value="192.168.56.102",
            description="IP address by which bob can be reached.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "config",
                    "combined_controllers.yaml",
                ]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_dual_robot_cell_control"),
                    "launch",
                    "rsp.launch.py",
                ]
            ),
            description="Launchfile (absolute path) providing the description. "
            "The launchfile has to start a robot_state_publisher node that "
            "publishes the description topic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_use_mock_hardware",
            default_value="false",
            description="Start alice with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_use_mock_hardware",
            default_value="false",
            description="Start bob with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for alice's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for bob's sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode for robot control for both arms.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_initial_joint_controller",
            default_value="alice_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the alice robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_initial_joint_controller",
            default_value="bob_scaled_joint_trajectory_controller",
            description="Initially loaded robot controller for the bob robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the alice robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller for the bob robot arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_dual_robot_cell_description"), "rviz", "urdf.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "alice_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bob_launch_dashboard_client",
            default_value="true",
            description="Launch Dashboard Client?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="update_rate_config_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("my_dual_robot_cell_control"),
                        "config",
                    ]
                ),
                "/",
                "update_rate.yaml",
            ],
        )
    )
    return LaunchDescription(declared_arguments + launch_setup())
