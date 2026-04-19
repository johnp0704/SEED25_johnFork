# waypoint_manager_node.py
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev
import numpy as np

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        self.GOAL_THRESH = 0.15
        self.current_waypoint_idx = 0

        self.mission_started = False
        self.get_logger().info("Waiting for start signal. Run: ros2 topic pub --once /start_mission std_msgs/msg/Empty {}")
        self.create_subscription(Empty, '/start_mission', self.start_callback, 10)

        raw_waypoints = [
            (0.9648,0.69976),
            (0.09597,0.72418),
            (-0.4974,0.67635),
            (-1.1528,0.65479),
            (-1.52718,0.50588),
            (-1.5958,0.17548),
            (-1.33124,-0.14362),
            (-0.84188,-0.10136),
            (-0.01673,-0.09453),
            (0.83987,-0.10445),
            (1.14145,-0.60751),
            (0.48705,-0.936818),
            (-0.957633,-0.95586),
            (-1.58721,-0.987335),
            (0.9648,0.69976),
        ]

        pts = np.array(raw_waypoints)
        tck, u = splprep([pts[:,0], pts[:,1]], s=0, k=2)
        u_new = np.linspace(u.min(), u.max(), 50)
        x_new, y_new = splev(u_new, tck)
        self.waypoints = list(zip(x_new, y_new))
        self.get_logger().info(f"Generated smooth spline with {len(self.waypoints)} points.")

        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.target_pub = self.create_publisher(Point, '/robot/current_target', 10)
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_bodies_callback, 10)

        self.robot_pose = None

    def start_callback(self, msg):
        if not self.mission_started:
            self.mission_started = True
            self.get_logger().info("Start command received! Beginning waypoint navigation.")

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
        if not self.mission_started or self.robot_pose is None:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!", throttle_duration_sec=2.0)
            return

        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        dx = target_x - self.robot_pose.x
        dy = target_y - self.robot_pose.y
        dist_to_goal = np.sqrt(dx**2 + dy**2)

        # ADD THIS — will print at most once per second so it doesn't spam
        self.get_logger().info(
            f"Waypoint idx={self.current_waypoint_idx}, "
            f"robot=({self.robot_pose.x:.3f},{self.robot_pose.y:.3f}), "
            f"target=({target_x:.3f},{target_y:.3f}), "
            f"dist={dist_to_goal:.3f}",
            throttle_duration_sec=1.0
        )

        if dist_to_goal < self.GOAL_THRESH:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached!")
                return
            target_x, target_y = self.waypoints[self.current_waypoint_idx]

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