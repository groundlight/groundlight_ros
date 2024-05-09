# Using OSRF's ROS Humble image with desktop tools
FROM osrf/ros:humble-desktop

# Avoid prompts from apt during build
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list
RUN apt-get update && \
    # Install development and wget tools
    apt-get install -y \
        ros-dev-tools \
        wget && \
    # Add the Gazebo package source
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    # Add the Gazebo repository key
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    # Update the package list again after adding new repository
    apt-get update && \
    # Install Ignition Fortress and Turtlebot4 simulator
    apt-get install -y \
        ignition-fortress \
        ros-humble-turtlebot4-simulator

# Install NVIDIA software
RUN sudo apt-get update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Set the noninteractive environment back to normal
ENV DEBIAN_FRONTEND=dialog

# Source the ROS2 environment in the bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
