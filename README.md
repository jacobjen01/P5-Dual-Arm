# P5-Dual-Arm
Semester project P5 gruppe 141 AAU Dual Arm

##Connection to the system
Connect to the router TPLINK -2.4 GHz(password is 29171798), add manual addresses to the settings:
address 192.168.56.<choose your number>, netmask 255.255.255.0, gateway 192.168.0.1
create ssh connection for the minipc with our study group number:

```bash
ssh -X gr141@192.168.56.2 
```

Kode til pc: gr141

Password login should be turned off so if you have not yet set up ssh with a key for your pc do this:

Minipc:
```bash
sudo nano /etc/ssh/sshd_config
```
Remove the # in 
```bash
PasswordAuthentication yes
```

On your system:

Check if you have a pubkey. Should have this written in it somewhere: ed25519
```bash
ls .ssh 
```
If you do, then ignore this step, but if you do not:
```bash
ssh-keygen -t ed25519
```

then
```bash
ssh-copy-id gr141@192.168.56.2
```
And write the password.

IMPORTANT: Go back in the config and comment out PasswordAuthentication yes

Now you can remote into the minipc with
```bash
ssh -X gr141@192.168.56.2 
```

## Prerequisites
This system is designed for Linux only and requires Docker with Compose v2.

### Installing Docker and Docker Compose v2
1. Install Docker:
```bash
sudo apt update
sudo apt install docker.io
```

2. Add your user to the docker group (to run Docker without sudo):
```bash
sudo usermod -aG docker $USER
```

3. Install Docker Compose v2:
```bash
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

4. **Important:** Log out and log back in (or restart) for the docker group membership to take effect.

5. Verify installation:
```bash
docker --version
docker compose version
```

## Docker Setup
The whole system runs in a docker container. Follow these steps:

### 1. Build the Docker Image
Before running for the first time, build the Docker image:
```bash
docker build -t p5_dual_arm .
```

### 2. Start the Container
```bash
docker compose up
```

### 3. Enable Display Support
To get GUI applications working in the container, run this command **after** the container is up:
```bash
xhost +local:root 
```

### 4. Test Display
To verify that the display is working, enter the container and test:
```bash
docker exec -it p5-dual-arm-dual-arm-1 bash
xeyes
colcon build
```
This should show a pair of eyes. Press Ctrl+C to close it again.

### 5. From now on a script can be used to make a container
```bash
./make_container.sh
```

## Robot control
### Launch the controller
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
### Move the robot
#### Using the pre defined poses
To move the robot to the predefined poses that is saved in 'pre_config_poses.json' use the following command.
```
ros2 service call /p5_move_to_pre_def_pose p5_interfaces/srv/MoveToPreDefPose "{robot_name: 'alice', goal_name: 'ALICE_HOME'}"
```
In the previous command alice can be replaced with the robot you want to move,
and ALICE_HOME can be replaced with the goal you want.

## Robot configurations 
### Configure predefined poses
To configure predefined poses run the following service call.
```
ros2 service call /p5_pose_config p5_interfaces/srv/PoseConfig "{pose: pose_name, joints: [j1, j2, j3, j4, j5, j6]}"
```
This saves the new joint angles to a json file. If the pose name already exists the pose will be updated.
If the pose dose not exists a new pose will be added to the file.

### Open and close end-effector
To open and close the end-effector run the following service call. Pin 16 is close when true and pin 17 is open when true. State is 0 or 1
```
ros2 service call /bob_io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: <state>}"
```
State must be set to ether 1 or 0 depending on if the end-effector needs to open or close.
NOTE it is not tested on the physical robot, so the pin might not be 17 and which state is open and which
is close is not known.
