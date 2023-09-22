# Use an official Ubuntu 22.04 base image that is ARM compatible
FROM arm64v8/ubuntu:22.04

# Set the maintainer label
LABEL maintainer="timmarkhuff@gmail.com"

# Avoid tzdata asking for geographic area and other interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Update the system and install necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    nano \
    curl \
    wget \
    sudo \
    locales \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository -y universe && \
    apt-get update && \
    apt-get upgrade -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]
