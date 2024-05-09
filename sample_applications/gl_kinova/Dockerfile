FROM osrf/ros:rolling-desktop

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# # Kinova installation instructions adapted from https://github.com/Kinovarobotics/ros2_kortex?tab=readme-ov-file
# RUN apt-get update && apt-get install -y \
#     python3-colcon-common-extensions \
#     python3-vcstool \
#     ros-humble-moveit \
#     ros-humble-rmw-cyclonedds-cpp \
#     ros-humble-rviz2 \
#     git \
#     && rm -rf /var/lib/apt/lists/*

# # Additional dependencies for GUI applications
# RUN apt-get update && apt-get install -y \
#     x11-apps \
#     mesa-utils && \
#     rm -rf /var/lib/apt/lists/*

# # Optional: Depending on the Kinova arm configuration, install the corresponding package
# # Uncomment the necessary line below based on your arm configuration:
# RUN apt-get update && apt-get install -y ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config
# # RUN apt-get update && apt-get install -y ros-humble-kinova-gen3-6dof-robotiq-2f-85-moveit-config

# # Set environment variables for DDS
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# # Set the workspace environment variable and directory
# ENV COLCON_WS=/workspace/ros2_kortex_ws
# RUN mkdir -p $COLCON_WS/src
# WORKDIR $COLCON_WS

# # Clone the necessary repositories
# RUN git clone https://github.com/PickNikRobotics/ros2_kortex.git src/ros2_kortex

# # Change to the directory containing the .repos files
# WORKDIR $COLCON_WS/src/ros2_kortex

# # Import all repos specified in the .repos files
# RUN vcs import ../ --skip-existing --input ros2_kortex-not-released.humble.repos && \
#     vcs import ../ --skip-existing --input ros2_kortex.humble.repos && \
#     vcs import ../ --skip-existing --input simulation.humble.repos

# # Change back to the workspace directory
# WORKDIR $COLCON_WS

# # Install ROS dependencies with rosdep
# RUN rosdep update && \
#     rosdep install --from-paths src --ignore-src -y -r

# # Build non-Kortex and problematic packages
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     colcon build --packages-skip-regex '.*kortex.*' '.*gen3.*' 'ros2_control_demo_example_12'"

# # Then build Kortex packages
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     colcon build --packages-select-regex '.*kortex.*' '.*gen3.*'"

# Source the ROS2 environment in the bashrc
RUN echo "source /opt/ros/rolling/setup.bash" >> /root/.bashrc




