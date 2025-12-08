import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_safety'),
                'launch',
                'launch_safety.launch.py'
            )
        )
    )
    launch_admittance_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_admittance_control'),
                'launch',
                'launch_admittance_node.launch.py'
            )
        )
    )
    launch_config_controler_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_config_control'),
                'launch',
                'launch_config_control_node.launch.py'
            )
        )
    )
    launch_relative_mover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_relative_mover'),
                'launch',
                'launch_relative_mover_node.launch.py'
            )
        )
    )
    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_launch'),
                'launch',
                'launch_future_estimator.py'
            )
        )
    )
    launch_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('p5_executor'),
                'launch',
                'launch_executor.launch.py'
            )
        )
    )

    return LaunchDescription([
        launch_safety,
        launch_admittance_node,
        launch_config_controler_node,
        launch_relative_mover,
        launch_camera,
        launch_executor
    ])
