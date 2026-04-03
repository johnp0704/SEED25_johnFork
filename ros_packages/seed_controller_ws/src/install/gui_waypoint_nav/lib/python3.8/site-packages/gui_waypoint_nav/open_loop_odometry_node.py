import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
import numpy as np
import time

class OpenLoopOdometryNode(Node):
    def __init__(self):
        super().__init__('open_loop_odometry_node')
        self.get_logger().info("Starting Open-Loop Dead Reckoning Virtual Sensor")

        # --- Robot State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Current motor speeds
        self.v_left = 0.0
        self.v_right = 0.0
        self.last_time = time.time()

        # --- Kinematic Calibration Constants ---
        # You will need to tune these later! 
        # They convert the Sabertooth command (-30 to 30) into actual Meters Per Second
        self.SPEED_TO_MPS_FACTOR = 0.02 
        self.L = 0.178 # Half the distance between the wheels in meters (from your follower script)

        # --- Pubs & Subs ---
        self.pose_pub = self.create_publisher(Pose2D, '/robot/pose2d', 10)
        self.create_subscription(Vector3, '/motor_cmds', self.cmd_callback, 10)
        self.create_subscription(Pose2D, '/reset_pose', self.reset_callback, 10)

        # Run the math loop at 20Hz
        self.create_timer(0.05, self.integrate_position)

    def cmd_callback(self, msg):
        # Convert the raw Sabertooth commands to estimated Meters Per Second
        self.v_left = msg.x * self.SPEED_TO_MPS_FACTOR
        self.v_right = msg.y * self.SPEED_TO_MPS_FACTOR

    def reset_callback(self, msg):
        self.get_logger().info("RE-HOME RECEIVED: Resetting odometry to origin (0,0,0)")
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def integrate_position(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Differential Drive Forward Kinematics
        # 1. Calculate linear velocity (v) and angular velocity (w)
        v = (self.v_right + self.v_left) / 2.0
        w = (self.v_right - self.v_left) / (2.0 * self.L)

        # 2. Integrate to find new position
        self.theta += w * dt
        
        # Wrap theta to keep it cleanly between -pi and pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt

        # 3. Publish to the network
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