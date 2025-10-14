# # Use the official ROS 2 Jazzy base image
# FROM ros:jazzy-ros-base

# # Set noninteractive frontend
# ENV DEBIAN_FRONTEND=noninteractive

# # Setup locales
# RUN apt-get update && apt-get install -y locales \
#     && locale-gen en_US en_US.UTF-8 \
#     && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
#     && export LANG=en_US.UTF-8

# # Install necessary packages
# RUN apt-get update && apt-get install -y \
# 	build-essential \
# 	ros-jazzy-ur-client-library \
# 	ros-jazzy-ur-simulation-gz \
# 	ros-jazzy-ur \
# 	ros-jazzy-ros2controlcli \
# 	ros-jazzy-rqt \
# 	ros-jazzy-moveit \
# 	ros-jazzy-ros2-control \
# 	python3-vcstool \
# 	vim \
# 	nano \
# 	git \
# 	iputils-ping \
# 	python3-colcon-common-extensions \
# 	x11-apps \	
# 	&& rm -rf /var/lib/apt/lists/*

# CMD xeyes
# # Source ROS 2 setup on container start
# SHELL ["/bin/bash", "-c"]
# RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 

# #CMD ["bash", "source /opt/ros/jazzy/setup.bash", "cd home/P5-Dual-Arm", "source install/setup.bash"]
# CMD ["bash"]


# ==========================================================
# ROS 2 Jazzy + Universal Robots + Intel RealSense
# ==========================================================

# Use the official ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# ----------------------------------------------------------
# 1. Locales
# ----------------------------------------------------------
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# ----------------------------------------------------------
# 2. Core dependencies (UR, MoveIt, tools)
# ----------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    ros-jazzy-ur-client-library \
    ros-jazzy-ur-simulation-gz \
    ros-jazzy-ur \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rqt \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    python3-vcstool \
    vim \
    nano \
    git \
    iputils-ping \
    python3-colcon-common-extensions \
    x11-apps \
    udev \
    libssl-dev \
    libusb-1.0-0-dev \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    cmake \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------
# 3. Install Intel RealSense SDK (librealsense2)
# ----------------------------------------------------------
WORKDIR /tmp
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/librealsense

# ----------------------------------------------------------
# 4. Setup ROS2 workspace and clone realsense-ros
# ----------------------------------------------------------
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/ros2_ws
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}/src

# Clone RealSense ROS2 wrapper (use ros2-development branch)
RUN git clone https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && \
    git checkout ros2-development || true

# ----------------------------------------------------------
# 5. Install ROS dependencies & build
# ----------------------------------------------------------
WORKDIR ${WORKSPACE}
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# ----------------------------------------------------------
# 6. Source ROS setup on startup
# ----------------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc

# ----------------------------------------------------------
# 7. Default command
# ----------------------------------------------------------
CMD ["bash"]
