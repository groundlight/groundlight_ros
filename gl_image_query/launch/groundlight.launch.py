from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ConditionalInclude
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Declare the camera_topic argument
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='',
        description='Topic for the camera')

    # Node for the image_query_action_server
    action_server = Node(
        package='gl_image_query',
        executable='image_query_action_server',
        name='image_query_action_server',
        output='screen')

    # Node for the framegrab, conditionally launched
    framegrab = Node(
        package='gl_framegrab',
        executable='framegrab',
        name='framegrab',
        output='screen',
        parameters=[{'camera_topic': LaunchConfiguration('camera_topic')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('camera_topic'), "' != ''"])))

    return LaunchDescription([
        camera_topic_arg,
        action_server,
        ConditionalInclude(condition=IfCondition(PythonExpression(["'", LaunchConfiguration('camera_topic'), "' != ''"])), 
                           launch_description_obj=framegrab)
    ])
