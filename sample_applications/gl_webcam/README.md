# Groundlight ROS with a Webcam

## Overview
This package is a simple example of using Groundlight ROS with just your webcam, no robot or simulation environment needed.

This guide assumes that you have already installed Groundlight ROS, sourced your installation of ROS2 and sourced your workspace. If you have not, please do so first. The [installation instructions](../../README.md) contain some useful tips on these points. 

## To Use
Bring up the Groundlight action server, camera server, and other imporant nodes by running `ros2 launch gl_webcam bringup.launch.py`

Run the demo with `ros2 run gl_webcam demo --ros-args -p query:="Is the person giving a thumbs up?"`. Press the enter key in this terminal to submit new image queries. 

You can replace the query with whatever question you would like to ask. Groundlight will create a custom 'detector' to answer your question. At first, it will take a bit of time to get an answer, since a human in the cloud must provide an answer, but over time, you will get faster and faster answers as the ML takes over. Please note: in this demo, a new detector is created each time your run the program for simplicity's sake. In an actual application, you would 
reuse detectors. 