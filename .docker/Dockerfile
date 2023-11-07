# Use an official Ubuntu 22.04 base image that is ARM compatible
FROM arm64v8/ubuntu:22.04

LABEL maintainer="tim@groundlight.ai"

# Avoid tzdata asking for geographic area and other interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Update the system and install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        nano \
        curl \
        locales \
        software-properties-common \
        python3-pip && \
    pip install groundlight && \
    rm -rf /var/lib/apt/lists/*

# Prepare to install ROS and install ROS
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository -y universe && \
    apt-get update && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    # Install ROS and colcon package manager
    apt-get install -y --no-install-recommends ros-humble-desktop python3-colcon-common-extensions python3-rosdep && \
    rm -rf /var/lib/apt/lists/* && \
    # Source the ROS environment and add some environment variables for networking
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    rosdep init

# Run `rosdep update` without sudo, which is recommended by ROS
RUN rosdep update

# Install UR ROS2 driver
RUN apt-get update && \
    apt-get install -y ros-humble-ur ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]
