# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Setup locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Install necessary packages
RUN apt-get update && apt-get install -y \
	build-essential \
	ros-jazzy-ur-client-library \
	ros-jazzy-ur-simulation-gz \
	ros-jazzy-ur \
	ros-jazzy-ros2controlcli \
	ros-jazzy-rqt \
	ros-jazzy-moveit \
	python3-vcstool \
	vim \
	nano \
	git \
	iputils-ping \
	python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/*

CMD xeyes
# Source ROS 2 setup on container start
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 

CMD ["bash"]
