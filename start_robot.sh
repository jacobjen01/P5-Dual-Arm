#!/bin/bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.51 headless_mode:=true
