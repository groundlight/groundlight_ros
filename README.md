# Groundlight ROS

Groundlight ROS makes it simple to integrate reliable visual applications into any robot with ROS 2. Your robot can ask questions about images and get timely, reliable answers. This can be useful for performing inspections or improving the decision making abilities of your robot. 

To learn more about Groundlight, visit our website: https://www.groundlight.ai/

## How does it work?

You simple ask binary (Yes/No) questions in natural language. Groundlight will automatically build a CV model to answer your question, based on the images you provide.  If the CV model is confident, you will get a quick response. If the model is unsure, it will escalate the image-query to a human monitor who will review the image and answer your question in real time (typically under a minute).  In this way you can always get reliable answers, even without a pre-existing training data set.

Also, every human answer (whether it's from you or a Groundlight Cloud Labeler) is added to a dataset used to train the CV model.  This means your model quickly "learns on the job" and will continue to improve in speed and accuracy over time.

If your question is unclear or ambiguous, the Cloud Labelers might not be able to answer your question.  In this case the image-query will be further escalated back to you, and show up as a "Flagged" image-query on your Dashboard.  It's important to answer Flagged image-queries, because otherwise the system will not know how to handle edge cases, which always come up in the real world.

## Requirements
This library requires ROS 2 Humble. If you don't already it, first [install ROS2 Humble](https://docs.ros.org/en/humble/Installation.html).

## Installation
If this is your first time using ROS, we recommend that you check out some of the [basic ROS tutorials](https://docs.ros.org/en/humble/Tutorials.html) before proceeding.

Once you have your ROS workspace configured, follow these steps:
1. Clone this repo into the src folder of your ROS 2 workspace: `git clone https://github.com/groundlight/groundlight_ros /path/to/your/workspace/src`.
2. From the root directory of your workspace, run: `colcon build`.
3. Source your installation of ROS 2. Again from the root directory of your workspace, run: `source install/setup.bash`.

## Authentication
To use Groundlight ROS, you must have a Groundlight account. If you don't already have a Groundlight account, you can [register for a free account here](https://dashboard.groundlight.ai/). 

Next, you need to obtain an API token. Log into your Groundlight account, go to the [Api Tokens](https://app.groundlight.ai/reef/my-account/api-tokens) page and generate a token. You can read [more about API tokens](https://code.groundlight.ai/python-sdk/docs/getting-started/api-tokens) on our site.

On the system where you are running ROS, run: `export GROUNDLIGHT_API_TOKEN="<YOUR API TOKEN>"`.

Some users prefer to put the token into their bashrc file so that it is always ready to be used. To do this, run: `echo 'export GROUNDLIGHT_API_TOKEN="<YOUR API TOKEN>"' >> ~/.bashrc && source ~/.bashrc`.

# A Simple Demo
To try a minimal example with Groundlight ROS on your laptop, you can run: `ros2 launch gl_webcam bringup.launch.py` to launch the demo application, and then `ros2 run gl_webcam webcam_example --ros-args -p query:="Is the person giving a thumbs up?"` to ask a question. Feel free to adjust the query to whatever you like. 

This sample application will create a brand-new Groundlight detector for you. Therefore, the response times will be slow at first. After submitting several image queries, your model will become more robust and return results much more quickly.

## Usage
This library assumes that your robot already has a topic where camera frames are published frequently and that the messages are of type `sensor_msgs/Image`. If this is not case, you may need to create a node to publish to such a topic.

Make note of the topic where your robot is publishing camera frames. If you are unsure, start up your robot and run: `ros2 topic list`. Browse the topics and find the one that seems right. Verify the type by running `ros2 topic type <YOUR CAMERA TOPIC>`.

To launch the Groundlight Image Query Action Server, run: `ros2 launch gl_image_query groundlight.launch.py camera_topic:=/<YOUR CAMERA TOPIC>`.

To see some examples of how to submit a Groundlight Image Query in ROS2, you can browse some of our [Sample Applications](https://github.com/groundlight/groundlight_ros/tree/main/sample_applications).
