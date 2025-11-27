from launch import LaunchDescription
from launch_ros.actions import Node
import math


def generate_launch_description():
    return LaunchDescription([
        #
        # Realsense D435i
        #
       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_alice_eef_link_to_at',
            arguments=[
                '0.1', '0.0', '0.25',  # x y z (meters)
                '0.0', '0.0', '3.14159265',  # roll pitch yaw (radians)
                'alice_tool0',  # parent frame
                'target'  # child frame
            ]
        ),

    ])
