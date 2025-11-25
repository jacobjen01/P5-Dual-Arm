# P5-dual-arm
this is a guide to show how to connect and use the MiR.

# first the MiR web gui.

# MiR network:
name: mir_250
password: mirex4you

# MiR web gui
once connected to the network go to any browser(google chrome is recommended) and enter the mir ip.
MiR ip: 192.168.12.20

(recommended)
it is also possible to be connected to the tp-Link network but then the ip is
MiR ip tp-Link: 192.168.x.x

# login for web gui mir
you will have to enter the name and password for a user.
username: distributor
password: distributor

# want more acces use:
username: admin
password: admin


now you are ready to use the web gui for mir 250.

usefull information to use in the web gui:
clicking the (?) icon on the left panel you can go to the mirs apis. the login is the same as when you enter the web gui(distributor). here you can get info and details about the mir.
(?)->api


# What to write in terminal
this next session will show how to get data from the MiR(maybe publish data to the MiR also¯\_(ツ)_/¯)

Run node and change endpoints - commands for ubuntu (ROS2):


# start ros2-node:
when running the ros2 node, write the usal run from the sorced file.

run ros2 pp5_mir_control mir_control_node

This will get the base, which is the ip adress(192.168.x.x), then the first line of code for the endpoint which will get system info, making the full code:

"http://192.168.x.x/api/v2.0.0/system/info"

it is possible to set own args for endpoint based on what api needs to be executed, --ros-args -p endpoint:="/api/v2.0.0/x/x"

the possible args to use are:
        "system_info": "/api/v2.0.0/system/info",
        "system_state": "/api/v2.0.0/system/state",
        "missions": "/api/v2.0.0/missions",
        "locations": "/api/v2.0.0/positions",
        "status": "/api/v2.0.0/status",
        "queue_drive_linear": "/api/v2.0.0/mission_queue/71319a5c-c473-11f0-ae86-000e8e983e91",
        "queue_charge_mir": "/api/v2.0.0/mission_queue/f6a377de-c475-11f0-ae86-000e8e983e91",

# Start ros2-node:
ros2 run p5_mir_control mir_control_node --ros-args -p endpoint:="/api/v2.0.0/system/info"
# Or use endpoint-keys:
ros2 run p5_mir_control mir_control_node --ros-args -p endpoint:=system_info
# Start with python (hurtig test):
python3 /home/jon-lap/P5-Dual-Arm/src/p5_mir_control/p5_mir_control/mir_control_node.py

# Change endpoint dynamisk while the node is running:
# Use the key for the ENDPOINTS:
ros2 param set /mir_rest_node endpoint system_state
# Use a path:
ros2 param set /mir_rest_node endpoint "/api/v2.0.0/system/state"
# Or change base+path with the full URL:
ros2 param set /mir_rest_node endpoint "http://192.168.100.253/api/v2.0.0/system/state"
# Check the parameter that is running now / node:
ros2 param get /mir_rest_node endpoint
ros2 node list
ros2 node info /mir_rest_node




