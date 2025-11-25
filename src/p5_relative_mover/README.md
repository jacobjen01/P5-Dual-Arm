# Relative mover
To move the robot to a pose using relative mover run the following command.
```
ros2 service call /alice/p5_move_to_pose p5_interfaces/srv/MoveToPose \
"{pose: {position: {x: x_pose, y: y_pose, z: z_pose}, orientation: {x: x_ori, y: y_ori, z: z_ori w: 1.0}}, linear: \
false, use_tracing_velocity: false, frame: 'frame'}"
```
To move bob just replace alice with bob.

# TO DO
Implement a way to disable relative mover via service call.
