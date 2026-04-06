import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Empty
import numpy as np
import time
import os
import json

class OpenLoopOdometryNode(Node):
    def __init__(self):
        super().__init__('open_loop_odometry_node')
        self.get_logger().info("Starting Open-Loop Dead Reckoning Virtual Sensor")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.last_time = time.time()

        # Dynamic Kinematics
        self.SPEED_TO_MPS_FACTOR = 0.02 
        self.L = 0.178
        self.load_calibration()

        # Pubs & Subs
        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.create_subscription(Vector3, '/motor_cmds', self.cmd_callback, 10)
        self.create_subscription(Pose2D, '/reset_pose', self.reset_callback, 10)
        self.create_subscription(Empty, '/calibration_updated', self.reload_callback, 10)

        self.create_timer(0.05, self.integrate_position)

    def load_calibration(self):
        calib_file = os.path.expanduser('~/.ros/seed_calibration.json')
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                    self.SPEED_TO_MPS_FACTOR = data.get('SPEED_TO_MPS_FACTOR', 0.02)
                    self.L = data.get('L', 0.178)
                self.get_logger().info(f"Loaded Kinematics: FACTOR={self.SPEED_TO_MPS_FACTOR:.6f}, L={self.L:.6f}")
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration: {e}")

    def reload_callback(self, msg):
        self.load_calibration()

    def cmd_callback(self, msg):
        self.v_left = msg.x * self.SPEED_TO_MPS_FACTOR
        self.v_right = msg.y * self.SPEED_TO_MPS_FACTOR

    def reset_callback(self, msg):
        self.get_logger().info("RE-HOME RECEIVED: Resetting odometry to origin (0,0,0)")
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.v_left = 0.0
        self.v_right = 0.0

    def integrate_position(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        v = (self.v_right + self.v_left) / 2.0
        w = (self.v_right - self.v_left) / (2.0 * self.L)

        self.theta += w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt

        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()