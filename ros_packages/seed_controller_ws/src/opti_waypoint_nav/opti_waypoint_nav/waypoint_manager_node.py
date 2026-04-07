import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Empty 
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev  # <-- NEW: Spline libraries
import numpy as np

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        # Loosened threshold: Since the points are so close together on a spline, 
        # a slightly larger threshold lets the robot fluidly "glide" through the curve 
        # rather than hunting for the exact center of every single dot.
        self.GOAL_THRESH = 0.15  
        self.current_waypoint_idx = 0 
        
        # --- Topic-based Startup Logic ---
        self.mission_started = False
        self.get_logger().info("Waiting for start signal. Run: ros2 topic pub --once /start_mission std_msgs/msg/Empty {}")
        self.create_subscription(Empty, '/start_mission', self.start_callback, 10)
        
        # --- NEW: Spline Generation Logic ---
        raw_waypoints = [
            (0.54899, -0.96740),
            (-0.56269, 0.74570),
            (-0.93254, -0.45896),
        ]

        pts = np.array(raw_waypoints)
        
        # Generate the spline. 
        # k=2 is a quadratic spline (used because we only have 3 points). 
        # If you add 4 or more points to your list later, change this to k=3 for an even smoother cubic spline!
        tck, u = splprep([pts[:,0], pts[:,1]], s=0, k=2)
        
        # Stretch those points into 50 intermediate points along the curve
        u_new = np.linspace(u.min(), u.max(), 50)
        x_new, y_new = splev(u_new, tck)
        
        # Zip them together into our final active waypoint list
        self.waypoints = list(zip(x_new, y_new))
        self.get_logger().info(f"Generated a smooth spline trajectory with {len(self.waypoints)} intermediate points.")
        # ------------------------------------

        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)

        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)

        self.robot_pose = None

    def start_callback(self, msg):
        """Triggers when a message is published to /start_mission"""
        if not self.mission_started:
            self.mission_started = True
            self.get_logger().info("Start command received! Beginning waypoint navigation.")

    def rigid_bodies_callback(self, msg):
        # Tracking rigid body '1'
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
        
        # Always publish the pose so the virtual twin updates immediately
        self.pose_pub.publish(self.robot_pose)

        self.update_waypoint()

    def update_waypoint(self):
        # Block target processing until the start topic is received
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
            # Note: I removed the "Reached waypoint X" print statement here. 
            # Since there are now 50 points, printing every single one will aggressively spam your terminal!
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