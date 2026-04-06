from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opti_waypoint_nav',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='virtual_twin_node',
            name='virtual_twin_node',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='dual_camera_node',
            name='dual_camera_node',
            output='screen'
        )
    ])