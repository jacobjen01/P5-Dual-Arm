from launch import LaunchDescription
from launch_ros.actions import Node
import math
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_launch = os.path.join(
        get_package_share_directory('p5_launch'),
        'launch',
        'launch_future_estimator.py'
        )
    
    ld = LaunchDescription([    

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
        ),
    
        Node(
            package='p5_controller',
            executable='move_to_pre_config_poses',
            name='move_to_pre_config_poses_node'
        ),



        Node(
            package='p5_controller',
            executable='relative_mover',
            name='relative_mover_node'
        ),

    ])

    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/robot_configurations ",
                "p5_interfaces/srv/RobotConfigurations ",
                "\"{robot_name: 'alice', goal_name: 'ALICE_HOME'}\"",
            ]],
            shell=True
        )
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/alice/servo_node/switch_command_type ",
                "moveit_msgs/srv/ServoCommandType ",
                "\"{command_type: 2}\"",
            ]],
            shell=True
        )
    )

    return ld

