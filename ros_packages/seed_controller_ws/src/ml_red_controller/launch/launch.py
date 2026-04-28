from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ml_red_controller',
            executable='ml_red_detector_node',
            name='ml_red_detector_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='gtg_controller_node',
            name='gtg_controller_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='gtg_motor_node',
            name='gtg_motor_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='cam_display_node',
            name='cam_display_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='arducam_detector_node',
            name='arducam_detector_node',
            output='screen'
        ),
    ])