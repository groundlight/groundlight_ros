from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='',
        description='Topic where your camera publishes images'
    )

    camera_topic = LaunchConfiguration('camera_topic')

    node1 = Node(
        package='gl_camera',
        executable='camera_server',
        name='camera_server',
        namespace='groundlight',
        parameters=[{'camera_topic': camera_topic}],
    )

    node2 = Node(
        package='gl_image_query',
        executable='action_server',
        name='action_server',
        namespace='groundlight',
    )

    node3 = Node(
        package='gl_image_query',
        executable='rviz_markers',
        name='rviz_markers',
        namespace='groundlight',
    )

    return LaunchDescription([
        camera_topic_arg,
        node1,
        node2,
        node3,
    ])
