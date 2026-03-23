from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opti_waypoint_nav',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='path_follower',
            name='path_follower',
            output='screen'
        )
    ])