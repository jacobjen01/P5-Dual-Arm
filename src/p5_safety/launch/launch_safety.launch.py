from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_safety',
            executable='_error_handling',
            output='screen',
        ),
        Node(
            package='p5_safety',
            executable='collision_detector_node',
            output='screen',
        ),
        Node(
            package='p5_safety',
            executable='robot_protective_stop_node',
            output='screen',
        ),
    ])
