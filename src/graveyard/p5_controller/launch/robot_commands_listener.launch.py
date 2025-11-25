from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_controller',
            executable='move_to_pre_config_poses',
            name='move_to_pre_config_poses',
            output='screen'
        ),
        Node(
            package='p5_controller',
            executable='admittance_node',
            name='alice_admittance_node',
            output='screen',
            parameters=[{'robot_name': 'alice'}]
        ),
        Node(
            package='p5_controller',
            executable='admittance_node',
            name='bob_admittance_node',
            output='screen',
            parameters=[{'robot_name': 'bob'}]
        )
    ])
