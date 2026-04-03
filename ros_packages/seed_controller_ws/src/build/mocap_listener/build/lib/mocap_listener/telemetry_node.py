#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
import matplotlib.pyplot as plt
import numpy as np
from math import atan2

# Configuration
ROBOT_NAME = '1'
GOAL_MARKER_INDEX = 5
REFRESH_RATE = 10  # Hz


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.subscription_rigid = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_callback,
            REFRESH_RATE)
        
        self.subscription_markers = self.create_subscription(
            Markers,
            '/markers',
            self.marker_callback,
            REFRESH_RATE)

        self.latest_rigid = None
        self.latest_markers = None

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.robot_scatter, = self.ax.plot([], [], 'bo', label='Robot Markers')
        self.goal_scatter, = self.ax.plot([], [], 'ro', label='Goal')
        self.heading_line, = self.ax.plot([], [], 'g-', label='Heading')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Real-Time Telemetry Visualization')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.axis('equal')

        # Update timer
        self.timer = self.create_timer(1.0 / REFRESH_RATE, self.update_plot)

    def rigid_callback(self, msg):
        self.latest_rigid = msg

    def marker_callback(self, msg):
        self.latest_markers = msg

    def update_plot(self):
        if self.latest_rigid is None or self.latest_markers is None:
            return

        # Get robot body
        robot_body = next((rb for rb in self.latest_rigid.rigidbodies if rb.rigid_body_name == ROBOT_NAME), None)
        if robot_body is None:
            self.get_logger().warn('Robot body not found')
            return

        # Extract marker positions
        robot_markers = []
        for mk in robot_body.markers:
            robot_markers.append([mk.translation.x, mk.translation.y])
        if len(robot_markers) < 2:
            self.get_logger().warn('Not enough robot markers to compute heading')
            return

        robot_markers = np.array(robot_markers)

        # Compute center
        center = np.mean(robot_markers, axis=0)

        # Compute heading vector (front direction)
        # Use first two markers if available to estimate orientation
        dx = robot_markers[0, 0] - robot_markers[1, 0]
        dy = robot_markers[0, 1] - robot_markers[1, 1]
        angle = atan2(dy, dx)

        # Get goal marker
        goal_marker = next((m for m in self.latest_markers.markers if m.marker_index == GOAL_MARKER_INDEX), None)
        if goal_marker is not None:
            goal_pos = np.array([goal_marker.translation.x, goal_marker.translation.y])
        else:
            goal_pos = np.array([np.nan, np.nan])

        # Update plots
        self.robot_scatter.set_data(robot_markers[:, 0], robot_markers[:, 1])
        self.goal_scatter.set_data(goal_pos[0], goal_pos[1])

        # Draw heading vector (arrow)
        heading_len = 0.2  # meters
        hx = [center[0], center[0] + heading_len * np.cos(angle)]
        hy = [center[1], center[1] + heading_len * np.sin(angle)]
        self.heading_line.set_data(hx, hy)

        # Adjust limits dynamically
        self.ax.set_xlim(center[0] - 1, center[0] + 1)
        self.ax.set_ylim(center[1] - 1, center[1] + 1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Optional: log to terminal
        self.get_logger().info(f"Pos=({center[0]:.2f},{center[1]:.2f}) | Angle={np.degrees(angle):.1f}° | Goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.close('all')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
