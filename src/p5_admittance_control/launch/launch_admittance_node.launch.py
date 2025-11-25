from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_admittance_control',
            executable='admittance_node',
            namespace='alice',
            output='screen',
            parameters=[{'robot_name': 'alice'}]
        ),
        Node(
            package='p5_admittance_control',
            executable='admittance_node',
            namespace='bob',
            output='screen',
            parameters=[{'robot_name': 'bob'}]
        )
    ])
