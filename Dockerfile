# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Setup locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    git wget curl gnupg2 lsb-release \
    build-essential cmake udev sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/

# Add Intel RealSense package repository
#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
#    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main"

# Build librealsense2 from source (for Ubuntu 24.04 / Jetson)
RUN apt-get update && apt-get install -y \
    git cmake libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev && \
    git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    mkdir build && cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/librealsense

# Install necessary packages
RUN apt-get update && apt-get install -y \
	build-essential \
	ros-jazzy-ur-client-library \
	ros-jazzy-ur-simulation-gz \
	ros-jazzy-ur \
	ros-jazzy-ros2controlcli \
	ros-jazzy-rqt \
	ros-jazzy-rqt-graph \
	ros-jazzy-moveit \
	ros-jazzy-moveit-servo \
	ros-jazzy-ros2-control \
	ros-jazzy-controller-manager \
	python3-vcstool \
	libeigen3-dev \
	libopencv-dev \
	libboost-all-dev \
	libpcl-dev \
	vim \
	nano \
	git \
	iputils-ping \
	python3-colcon-common-extensions \
	x11-apps \	
#    librealsense2-utils librealsense2-dev librealsense2-dkms \
    ros-jazzy-realsense2-camera \
    ros-jazzy-image-tools \
    ros-jazzy-apriltag-ros \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*


CMD xeyes
# Source ROS 2 setup on container start
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 

#CMD ["bash", "source /opt/ros/jazzy/setup.bash", "cd home/P5-Dual-Arm", "source install/setup.bash"]
CMD ["bash"]
