from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os

def generate_launch_description():
    # Retrieve the RViz config file path from the share directory
    pkg_share = get_package_share_directory('gl_turtlebot')
    rviz_config_path = os.path.join(pkg_share, 'config', 'demo.rviz')

    # Define the RViz launch action
    rviz_launch = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Define nodes based on the provided commands
    camera_server_node = launch_ros.actions.Node(
        package='gl_camera',
        executable='camera_server',
        name='camera_server',
        parameters=[{'camera_topic': '/oakd/rgb/preview/image_raw'}],
        output='screen'
    )

    action_server_node = launch_ros.actions.Node(
        package='gl_image_query',
        executable='action_server',
        name='action_server',
        output='screen'
    )

    rviz_markers_node = launch_ros.actions.Node(
        package='gl_image_query',
        executable='rviz_markers',
        name='rviz_markers',
        output='screen'
    )

    demo_node = launch_ros.actions.Node(
        package='gl_turtlebot',
        executable='demo',
        name='demo',
        output='screen'
    )

    # Include all nodes in the launch description
    return launch.LaunchDescription([
        rviz_launch,
        camera_server_node,
        action_server_node,
        rviz_markers_node,
        demo_node
    ])