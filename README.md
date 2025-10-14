# P5-Dual-Arm
Semester project P5 gruppe 141 AAU Dual Arm Beer Buddy

## Docker
The whole system runs in a docker container which can easily be started using. It is only for linux though.
```
docker compose up
```
For docker compose to work both docker and docker-compose-v2 has to be installed.
If the container has not been build then it can be build with the following command.
```
docker build -t p5_dual_arm .
```
To get a display in the container run.
```
xhost +local:root 
```
This can be done after the container is up.
To test if the display is working try to run.
```
xeyes
```
This should show a pair of eyes press Ctrl to close it again.



## Robot controller
The robot controller both be launched with the physical system and without the physical system.

To launch the controller with the physical system run the following command.
```
ros2 launch my_dual_robot_cell_control start_robots.launch.py
```
Then for launching the controller without the physical system, 
so in simulation mode, add the following to the previous command.
```
alice_use_mock_hardware:=true bob_use_mock_hardware:=true
```


