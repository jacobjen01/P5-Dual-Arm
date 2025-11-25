#! /bin/bash
ros2 service call /p5_move_to_pre_def_pose p5_interfaces/srv/MoveToPreDefPose "{robot_name: 'alice', goal_name: 'ALICE_HOME'}"
wait 30
ros2 service call /alice/servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
wait 30
ros2 service call /alice/p5_move_to_pose p5_interfaces/srv/MoveToPose "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, linear: false, use_tracking_velocity: false, frame: 'target'}"
