from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_config_control',
            executable='config_node_config',
            name='p5_config_control_config',
            output='screen'
        ),
        Node(
            package='p5_config_control',
            executable='config_node_controller',
            name='p5_config_control',
            output='screen'
        )
    ])
