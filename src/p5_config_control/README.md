# Config control
Config control moves the robot in joint space which is used for example to home the robots.
To use config control run the following command.
```
ros2 service call /p5_move_to_pre_def_pose p5_interfaces/srv/MoveToPreDefPose "{robot_name: 'robot_name', goal_name: 'goal_name'}"
```
This will return true when the goal is received, so to see when the robot is done executing, run the following command.
```
ros2 topic echo /p5_joint_mover_status 
```
When this prints true then the robot is done executing.

# Config control config
For the config control there is also a config control config node.
This node is for making new or updating poses and then save them in a json file.

