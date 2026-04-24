import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # The master multiplexer
        Node(
            package='demo_night',
            executable='commander_node',
            name='commander_node',
            output='screen'
        ),
        
        # The PyQt6 Interface and Virtual Twin
        Node(
            package='demo_night',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
        
        # ArUco Fiducial Rehoming Sequence
        Node(
            package='demo_night',
            executable='aruco_rehoming_node',
            name='aruco_rehoming_node',
            output='screen'
        ),
        
        # Blue Tape Optical Path Follower
        Node(
            package='demo_night',
            executable='optical_path_follower_node',
            name='optical_path_follower_node',
            output='screen'
        ),
        
        # Red Mask Go-To-Goal (Highest Priority Override)
        Node(
            package='demo_night',
            executable='gtg_controller_node',
            name='gtg_controller_node',
            output='screen'
        ),
        
        # Realsense controlling node
        Node(
            package='demo_night',
            executable='realsense_node',
            name='realsense_node',
            output='screen'
        ),
        Node(
            package='demo_night',
            executable='arducam_node',
            name='arducam_node',
            output='screen'
        ),
        # Cam display node
        Node(
            package='demo_night',
            executable='display_node',
            name='display_node',
            output='screen'
        ),
    ])