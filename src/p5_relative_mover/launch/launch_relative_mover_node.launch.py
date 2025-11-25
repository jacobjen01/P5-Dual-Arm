from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_relative_mover',
            executable='relative_mover_node',
            namespace='alice',
            output='screen',
            parameters=[{'robot_prefix': 'alice'}],
        ),
        Node(
            package='p5_relative_mover',
            executable='relative_mover_node',
            namespace='bob',
            output='screen',
            parameters=[{'robot_prefix': 'bob'}],
        )
    ])
