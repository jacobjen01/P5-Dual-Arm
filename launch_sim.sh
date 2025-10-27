!#/bin/bash

export ROS_DOMAIN_ID=69
source install/setup.bash
ros2 launch my_dual_robot_cell_control start_robots.launch.py alice_use_mock_hardware:=true bob_use_mock_hardware:=true
