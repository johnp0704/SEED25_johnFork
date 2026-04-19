from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # ==========================================
        # 1. OPTITRACK & WAYPOINT NAVIGATION NODES
        # ==========================================
        Node(
            package='opti_waypoint_nav',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            output='screen'
        ),
        Node(
            package='opti_waypoint_nav',
            executable='path_follower_node',  # The Master Motor Hub
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
        ),

        # ==========================================
        # 2. MACHINE VISION & GO-TO-GOAL NODES
        # ==========================================
        Node(
            package='ml_red_controller',
            executable='ml_red_detector_node',
            name='ml_red_detector_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='gtg_controller_node',  # The Vision Override
            name='gtg_controller_node',
            output='screen'
        ),
        Node(
            package='ml_red_controller',
            executable='cam_display_node',
            name='cam_display_node',
            output='screen'
        ),

        # ==========================================
        # 3. END EFFECTOR / TOOL CONTROLLER
        # ==========================================
        # Node(
        #     package='ml_red_controller',
        #     executable='front_controller_node', # The Auger ESP32 Controller
        #     name='front_controller_node',
        #     output='screen',
        #     parameters=[
        #         # Use this to hardcode the port if auto-select fails (e.g., '/dev/ttyUSB0')
        #         # {'serial_port': ''} 
        #     ]
        # )
    ])