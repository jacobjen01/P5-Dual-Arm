from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #
        # Realsense D435i
        #
        Node(
            package='realsense2_camera',
            namespace='camera',
            executable='realsense2_camera_node',
            name='camera'
        ),
        #
        # April tag reader
        #
        Node(
            package='apriltag_ros',
            namespace='apriltag',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
            parameters=[{
                'size': 0.077
            }]
        ),
        #
        # Future tag estimator
        #
        Node(
            package='p5_perception',
            namespace='perception',
            executable='future_tag_estimator',
            name='future_tag_estimator_node'
        ),

        # Static transform from mir -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_alice_base_link_to_camera_node',
            arguments=[
                '0.04', '-0.30', '0.66',  # x y z (meters)
                '-1.57079', '0', '0',        # roll pitch yaw (radians)
                'mir',          # parent frame
                'camera_link'         # child frame
            ]
        ),
    ])

