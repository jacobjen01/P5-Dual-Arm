FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
	build-essential \
	ros-jazzy-ur \
	ros-jazzy-ros2controlcli \
	vim \
	nano \
	git \
	iputils-ping \
	python3-colcon-common-extensions \
	&& rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 

CMD ["bash"]
