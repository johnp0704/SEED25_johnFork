import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from geometry_msgs.msg import Point, Pose2D
from scipy.spatial.transform import Rotation as R
import numpy as np

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        # Configuration
        self.GOAL_THRESH = 0.3  # meters
        self.current_waypoint_idx = 0  # Tracks which marker in the sorted list we are aiming for

        # Publishers
        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)

        # Subscribers
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)
        self.create_subscription(Markers, '/markers', self.markers_callback, 10)

        self.latest_markers = []
        self.robot_pose = None

    def markers_callback(self, msg):
        # Sort markers by their index so they are always in order (0, 1, 2, 3...)
        self.latest_markers = sorted(msg.markers, key=lambda m: m.marker_index)

    def rigid_bodies_callback(self, msg):
        # 1. Extract and process Robot Pose
        robot_body = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            return

        pos = robot_body.pose.position
        ori = robot_body.pose.orientation

        # Convert quaternion to Yaw
        r_mocap = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        R_correction = R.from_euler('z', -90, degrees=True)
        r_ros = R_correction * r_mocap
        _, _, yaw = r_ros.as_euler('xyz', degrees=False)

        # Save and publish pose for the controller node
        self.robot_pose = Pose2D()
        self.robot_pose.x = pos.x
        self.robot_pose.y = pos.y
        self.robot_pose.theta = yaw
        self.pose_pub.publish(self.robot_pose)

        # 2. Manage Waypoints
        self.update_waypoint()

    def update_waypoint(self):
        if not self.latest_markers or self.robot_pose is None:
            return

        # Check if we have completed all waypoints
        if self.current_waypoint_idx >= len(self.latest_markers):
            self.get_logger().info("All waypoints reached! Path complete.", throttle_duration_sec=2.0)
            return

        # Get the current target marker
        target_marker = self.latest_markers[self.current_waypoint_idx]

        # Calculate distance to current target
        dx = target_marker.translation.x - self.robot_pose.x
        dy = target_marker.translation.y - self.robot_pose.y
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        # If within threshold, advance to the next marker
        if dist_to_goal < self.GOAL_THRESH:
            self.get_logger().info(f"Reached marker {target_marker.marker_index}! Advancing to next waypoint.")
            self.current_waypoint_idx += 1
            return 

        # Publish the active target
        target_msg = Point()
        target_msg.x = target_marker.translation.x
        target_msg.y = target_marker.translation.y
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