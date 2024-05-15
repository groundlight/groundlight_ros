from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Retrieve the RViz config file path from the share directory
    pkg_share = get_package_share_directory('gl_webcam')
    rviz_config_path = os.path.join(pkg_share, 'config', 'demo.rviz')
    laptop_urdf_path = os.path.join(pkg_share, 'urdf', 'laptop.urdf')

    # Define the RViz launch action
    rviz_launch = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    webcam_node = launch_ros.actions.Node(
        package='gl_camera',
        executable='webcam',
        name='webcam',
        output='screen'
    )

    camera_server_node = launch_ros.actions.Node(
        package='gl_camera',
        executable='camera_server',
        name='camera_server',
        parameters=[{'camera_topic': '/groundlight/webcam_frames'}],
        output='screen'
    )

    action_server_node = launch_ros.actions.Node(
        package='gl_image_query',
        executable='action_server',
        name='action_server',
        output='screen'
    )

    robot_description = {
        'robot_description': ParameterValue(
            launch.substitutions.Command(['xacro ', launch.substitutions.TextSubstitution(text=laptop_urdf_path)]),
            value_type=str
        )
    }

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='gl_webcam',
        executable='joint_publisher',
        name='joint_publisher',
        output='screen'
    )

    frustum_publisher_node = launch_ros.actions.Node(
        package='gl_webcam',
        executable='frustum_publisher',
        name='frustum_publisher',
        output='screen'
    )

    image_drawer_node = launch_ros.actions.Node(
        package='gl_webcam',
        executable='image_drawer',
        name='image_drawer',
        output='screen'
    )

    return launch.LaunchDescription([
        rviz_launch,
        webcam_node,
        camera_server_node,
        action_server_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        frustum_publisher_node,
        image_drawer_node,
    ])