# Admittance control
All the example commands are with alice, so to do them with bob just replace alice with bob.
The relative mover node should publish a pose to the admittance node, 
if the admittance node has to be tested, then that can be done by the following command.
```
ros2 topic pub /alice/p5_robot_pose_to_admittance geometry_msgs/msg/PoseStamped "{header: {stamp: 'now', frame_id: 'frame'}, pose: {position: {x: x_pose, y: y_pose, z: z_pose}, orientation: {x: x_ori, y: y_ori, z: z_ori, w: 1.0}}}" --once
```
However the relative mover node has to be disabled first (see p5_relative_mover/README.md)
The pose is in meters, and the orientation is in quaternion.

## Configurations
There are some configurations that can be changed while the node is running. 
The configurations can be changed with the following command.
```
ros2 service call /alice/p5_admittance_config p5_interfaces/srv/AdmittanceConfig "{m: param, d: param, k: param, alpha: alpha}"
```
Where alpha is for the low pass filter, and the param is the following list.
```
[x_force, y_force, z_force, x_torque, y_torque, z_torque]
```
The admittance control can also be turned on and off with the following command.
```
ros2 service call /alice/p5_admittance_set_state p5_interfaces/srv/AdmittanceSetStatus "{active: false, update_rate: 250}"
```
Here the update rate can also be set.

## Save parameters 
The parameters can also be saved using the following command.
```
ros2 service call /alice/p5_save_admittance_param p5_interfaces/srv/SaveAdmittanceParam "{param_name: 'name'}"
```

## Show status
To show the status on admittance control run the following command.
```
ros2 service call /alice/p5_admittance_show_status p5_interfaces/srv/AdmittanceShowStatus
```


