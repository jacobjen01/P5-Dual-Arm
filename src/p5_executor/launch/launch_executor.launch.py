from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_executor',
            executable='program_executor_node',
            output='screen'
        )
    ])
