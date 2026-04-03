import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D, PoseArray
import numpy as np

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        self.get_logger().info("GUI Waypoint Manager Initialized")

        self.GOAL_THRESH = 0.3  # meters
        self.current_waypoint_idx = 0 
        self.active_waypoints = []
        self.robot_pose = None

        # Publishers & Subscribers
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)
        
        # Listens to the dead-reckoning node for the robot's physical location
        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)
        
        # Listens to the GUI for the list of clicked waypoints
        self.create_subscription(PoseArray, '/planned_waypoints', self.gui_callback, 10)

        # Core logic loop (Runs at 10Hz)
        self.create_timer(0.1, self.update_waypoint)

    def gui_callback(self, msg):
        # Unpack the ROS PoseArray into a simple Python list of (x, y) coordinates
        self.active_waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.current_waypoint_idx = 0
        self.get_logger().info(f"Received new mission with {len(self.active_waypoints)} targets.")

    def pose_callback(self, msg):
        # Update our internal knowledge of where the robot currently is
        self.robot_pose = msg

    def update_waypoint(self):
        # Don't do any steering math if we don't have a mission or don't know where the robot is
        if not self.active_waypoints or self.robot_pose is None:
            return

        # Check if the mission is complete
        if self.current_waypoint_idx >= len(self.active_waypoints):
            # Throttle the "done" message so it doesn't spam your terminal 10 times a second
            self.get_logger().info("All waypoints reached! Awaiting new mission.", throttle_duration_sec=5.0)
            return

        # Get the coordinate of the waypoint we are currently hunting
        target_x, target_y = self.active_waypoints[self.current_waypoint_idx]

        # Calculate Euclidean distance to the current target
        dx = target_x - self.robot_pose.x
        dy = target_y - self.robot_pose.y
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        # If we enter the threshold radius, advance to the next marker
        if dist_to_goal < self.GOAL_THRESH:
            self.get_logger().info(f"Waypoint {self.current_waypoint_idx + 1} Reached! Advancing...")
            self.current_waypoint_idx += 1
            return 

        # Keep publishing the active target point so the path_follower keeps its motors spinning
        target_msg = Point()
        target_msg.x = target_x
        target_msg.y = target_y
        target_msg.z = 0.0
        self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()