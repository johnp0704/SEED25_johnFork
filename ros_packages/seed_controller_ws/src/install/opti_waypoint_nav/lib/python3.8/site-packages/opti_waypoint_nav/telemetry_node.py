import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
import matplotlib.pyplot as plt
import numpy as np

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.get_logger().info("Starting Telemetry Dashboard")

        self.robot_pose = None
        self.current_goal = None

        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(4, 4))
        self.ax.axis('off') # Hide axes for a clean text dashboard
        
        self.textbox = self.ax.text(
            0.1, 0.5, "", transform=self.ax.transAxes, fontsize=12, va='center', ha='left'
        )
        self.create_timer(0.1, self.update_telemetry)

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.current_goal = msg

    def update_telemetry(self):
        if self.robot_pose and self.current_goal:
            x, y, yaw = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
            gx, gy = self.current_goal.x, self.current_goal.y
            
            dx = gx - x
            dy = gy - y
            dist = np.sqrt(dx**2 + dy**2)
            
            theta_des = np.arctan2(dy, dx)
            error_theta = theta_des - yaw
            error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

            info = (
                f"--- ROBOT STATE ---\n"
                f"X: {x:.2f} m\n"
                f"Y: {y:.2f} m\n"
                f"Heading: {np.degrees(yaw):.1f}°\n\n"
                f"--- TARGET STATE ---\n"
                f"Goal X: {gx:.2f} m\n"
                f"Goal Y: {gy:.2f} m\n"
                f"Dist to Goal: {dist:.2f} m\n"
                f"Heading Error: {np.degrees(error_theta_wrapped):.1f}°"
            )
            self.textbox.set_text(info)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()