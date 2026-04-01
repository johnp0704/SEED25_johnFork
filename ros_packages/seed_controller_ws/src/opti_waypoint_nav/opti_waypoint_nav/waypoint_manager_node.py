import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import Point, Pose2D
from scipy.spatial.transform import Rotation as R
import numpy as np
import threading  # <-- NEW: Imported threading

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        self.GOAL_THRESH = 0.3  # meters
        self.current_waypoint_idx = 0 
        
        # --- NEW: Manual Startup Logic ---
        self.mission_started = False
        self.get_logger().info("Node initialized. OptiTrack listening...")
        
        # Start a background thread to wait for user input so we don't block ROS callbacks
        threading.Thread(target=self.wait_for_start_command, daemon=True).start()
        # ---------------------------------

        self.waypoints = [
            (0.7328, -0.7006),
            (-0.3972, 0.27242),
            (-1.2768, -0.69690),
        ]

        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)

        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)

        self.robot_pose = None

    def wait_for_start_command(self):
        """Runs in a background thread. Waits for the user to press Enter."""
        # This will print to the terminal where you ran the node
        input("\n>>> PRESS ENTER TO START WAYPOINT NAVIGATION <<<\n\n")
        self.mission_started = True
        self.get_logger().info("Mission started! Now publishing targets to the robot.")

    def rigid_bodies_callback(self, msg):
        robot_body = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == '2'), None)
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
        
        # We ALWAYS publish the pose so the virtual twin can see the robot instantly
        self.pose_pub.publish(self.robot_pose)

        self.update_waypoint()

    def update_waypoint(self):
        # Block target processing if the user hasn't pressed Enter yet
        if not self.mission_started:
            return
            
        if self.robot_pose is None:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached! Path complete.", throttle_duration_sec=2.0)
            return

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