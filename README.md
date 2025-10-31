# P5-Dual-Arm
Semester project P5 gruppe 141 AAU Dual Arm Beer Buddy

##Connection to the system
Connect to the router TPLINK -2.4 GHz(password is 29171798), add manual addresses to the settings:
address 192.168.56.<choose your number>, netmask 255.255.255.0, gateway 192.168.0.1
create ssh connection for the jetson with our study group number:

```bash
gr-141@192.168.56.2 
```

Kode til Jetson: gr141

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
```
This should show a pair of eyes. Press Ctrl+C to close it again.



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

## Home
To home the robot use this
```
ros2 service call /robot_configurations p5_interfaces/srv/RobotConfigurations "{command: 'HOME'}"
```
