from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_waypoint_nav',
            executable='waypoint_gui_node',
            name='waypoint_gui_node',
            output='screen'
        ),
        Node(
            package='gui_waypoint_nav',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            output='screen'
        ),
        Node(
            package='gui_waypoint_nav',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),
        Node(
            package='gui_waypoint_nav',
            executable='dual_camera_node',
            name='dual_camera_node',
            output='screen'
        ),
        Node(
            package='gui_waypoint_nav',
            executable='open_loop_odometry_node',
            name='open_loop_odometry_node',
            output='screen'
        ),
    ])