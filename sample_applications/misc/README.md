# Universal Robots
Groundlight can enhance the operation of [Universal Robots](https://www.universal-robots.com/) arms by answering questions about image data in the robot's environment. Common applications are automated inspections, machine tending and anomoly detection.

## Gettings Started with Docker
We provide a docker image that makes using Groundlight with a UR robot easy. This Docker image is intended to be run on an Nvidia Jetson that connects to the robot with ethernet. If you don't want to use Docker, you can skip this step, but you will need to install the UR Drivers separately. To get started with Docker:
1. Build the docker image: `docker build -t ur_ros2_image .`
2. Set up the robot to talk to the Jetson. On the Jetson, add the following to /etc/network/interfaces:
```
iface eth1 inet static
    address 192.168.1.101
    netmask 255.255.255.0
```
3. On the robot's teach pendant, open the network settings (Setup Robot -> Network) and enter the IP addresses as shown at: https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html
4. Install the [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/foxy/ur_robot_driver/doc/install_urcap_e_series.md) on the robot

## Running with Docker
1. Run the container with `docker run --privileged --device /dev/video0 -v /home/rosie/ros2_ws:/root/ros2_ws -it --net=host ur_ros2_image`. It is necessary to set up port forwarding as seen in the preceding command so that the robot can communicate with the docker container. Also, it is useful to mount a drive so that the container gets access to files from the Jetson. The --device flag gives the container access to a camera that is plugged into the Jetson.
2. Inside the container, launch the UR driver: `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 reverse_ip:=192.168.1.101`. Note how it is necessary to specify a reverse IP address so that the Jetson can communicate with the robot.
3. On the robot's teach pendant, run the External Control URCap. Without this, ROS will not have permission to move the robot.
4. Try running a sample app (coming soon)

Usage
1. (coming soon)
