from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
            name='camera',
            parameters=[
                {'enable_color': True},        # Enable RGB camera
                {'enable_depth': False},       # Disable depth stream
                {'enable_infra1': False},      # Disable infrared 1
                {'enable_infra2': False},      # Disable infrared 2
                {'enable_fisheye1': False},    # Disable fisheye (if present)
                {'enable_fisheye2': False},    # Disable fisheye (if present)
                {'enable_gyro': False},        # Disable gyro
                {'enable_accel': False},       # Disable accelerometer
                {'enable_sync': True},        # Disable sync mode
                {'enable_pointcloud': False},  # Disable point cloud generation
                {'enable_confidence': False},  # Disable confidence stream
                {'enable_rgbd': False},        # Disable RGB-D composite
                {'enable_pose': True},        # Disable pose tracking
                {'enable_depth_infra': False}, # Disable depth-infra composite
                {'enable_auto_exposure': True},# Optional: keep auto exposure for color
                {'color_width': 640},          # Set resolution for color stream
                {'color_height': 480},         # Set resolution for color stream
                {'color_fps': 20},             # Set FPS for color stream
            ]
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
            name='future_tag_estimator_node',
            parameters=[{'use_averaging': True}, {'only_point': False}],
            #output='screen'
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

