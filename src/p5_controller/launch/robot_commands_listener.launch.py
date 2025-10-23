from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p5_controller',
            executable='home',
            name='robot_command_listener',
            output='screen'
        )
    ])