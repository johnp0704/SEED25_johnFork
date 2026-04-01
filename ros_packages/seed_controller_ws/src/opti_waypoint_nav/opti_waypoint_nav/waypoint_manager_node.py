import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies  # Removed Markers import
from geometry_msgs.msg import Point, Pose2D
from scipy.spatial.transform import Rotation as R
import numpy as np

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        self.GOAL_THRESH = 0.3  # meters
        self.current_waypoint_idx = 0 

        # Hardcoded waypoints list as (x, y) tuples
        # You can eventually replace this by subscribing to a GUI topic
        self.waypoints = [
            (0.7328, -0.7006),
            (-0.3972, 0.27242),
            (-1.2768, -0.69690),
        ]

        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)

        # Keep the rigid bodies subscription for robot localization
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)

        self.robot_pose = None

    def rigid_bodies_callback(self, msg):
        robot_body = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            return

        pos = robot_body.pose.position
        ori = robot_body.pose.orientation

        r_mocap = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        R_correction = R.from_euler('z', -90, degrees=True)
        r_ros = R_correction * r_mocap
        _, _, yaw = r_ros.as_euler('xyz', degrees=False)

        self.robot_pose = Pose2D()
        self.robot_pose.x = pos.x
        self.robot_pose.y = pos.y
        self.robot_pose.theta = yaw
        self.pose_pub.publish(self.robot_pose)

        self.update_waypoint()

    def update_waypoint(self):
        if self.robot_pose is None:
            return

        # Check if we have exhausted our list of waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Path complete.", throttle_duration_sec=2.0)
            return

        # Grab the target x and y from the hardcoded list
        target_x, target_y = self.waypoints[self.current_waypoint_idx]

        dx = target_x - self.robot_pose.x
        dy = target_y - self.robot_pose.y
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        if dist_to_goal < self.GOAL_THRESH:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}! Advancing...")
            self.current_waypoint_idx += 1
            return 

        target_msg = Point()
        target_msg.x = float(target_x)
        target_msg.y = float(target_y)
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