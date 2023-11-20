# Groundlight ROS

Groundlight ROS makes it simple to integrate reliable visual applications into any robot with ROS 2. Your robot can ask questions about images and get timely, high-confidence answers. This can be useful for performing inspections or improving the decision making abilities of your robot. 

To learn more about Groundlight, check out our website: https://www.groundlight.ai/

## Requirements
This library requires ROS 2 Humble. If you don't already it, first [install ROS2 Humble](https://docs.ros.org/en/humble/Installation.html).

## Installation
If this is your first time using ROS, we recommend that you check out some of the [basic ROS tutorials](https://docs.ros.org/en/humble/Tutorials.html) before proceeding.

Once you have your ROS workspace configured, follow these steps:
1. Clone this repo into the src folder of your ROS 2 workspace: `git clone git@github.com:groundlight/groundlight_ros.git /path/to/your/workspace/src`.
2. From the root directory of your workspace, run: `colcon build`.
3. Source your installation of ROS 2. Again from the root directory of your workspace, run: `source install/setup.bash`.

## Authentication
To use Groundlight ROS, you must be authenticated with Groundlight. If you don't already have a Groundlight account, you can [register here](https://www.groundlight.ai/Signup). 

Next, you need to obtain an API token. Log into your Groundlight account, go to the [Api Tokens](https://app.groundlight.ai/reef/my-account/api-tokens) page and generate a token. You can read [more about API tokens](https://code.groundlight.ai/python-sdk/docs/getting-started/api-tokens) on our site.

On the system where you are running ROS, run: `export GROUNDLIGHT_API_TOKEN="<YOUR API TOKEN>"`.

Some users prefer to put the token into their bashrc file so that it is always ready to be used. To do this, run: `echo 'export GROUNDLIGHT_API_TOKEN="<YOUR API TOKEN>"' >> ~/.bashrc && source ~/.bashrc`.

## Usage
This library assumes that your robot already has a topic where camera frames are published frequently and that the messages are of type `sensor_msgs/Image`. If this is not case, you may need to create node to publish to such a topic.

Make note of the topic where your robot is publishing camera frames. If you are unsure, start up your robot and run: `ros2 topic list`. Browse the topics and find the one that seems right. Verify the type by running `ros2 topic type <topic_name>`.

Now you're ready to try a minimal example with Groundlight ROS:
1. Launch your robot according to the instructions provided in the ROS 2 package for your robot. If there is a separate launch file for your robot's camera, launch that too.
2. Start the Groundlight action server: `ros2 launch gl_image_query groundlight.launch.py camera_topic:=<YOUR IMAGE TOPIC>`
3. Run the Groundlight ROS hello world program: `ros2 run gl_image_query hello_world`

This will run a simple example that takes a picture from your robot's camera and asks the question "Is there a person in this picture?" At first, the response times will be slow because your model is new, but after submitting several image queries, your model will become more robust and return results more quickly.

## Sample Applications
This library provides several sample applications of Groundlight being used with different robots. You can browse the `sample_applications` folder to get inspiration for how to use this library.

