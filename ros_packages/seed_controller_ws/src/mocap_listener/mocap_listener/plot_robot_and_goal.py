#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
import matplotlib.pyplot as plt
import numpy as np

GOAL_MARKER_INDEX = 5

class PlotRobotNode(Node):
    def __init__(self):
        super().__init__('plot_robot_node')
        self.get_logger().info("Starting robot + goal plotting node")

        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None

        # Subscriptions
        self.create_subscription(RigidBodies, '/rigid_bodies', self.rb_callback, 10)
        self.create_subscription(Markers, '/markers', self.marker_callback, 10)

        # Set up interactive plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))

        # Timer to update plot at ~10 Hz
        self.create_timer(0.1, self.update_plot)

    def rb_callback(self, msg):
        self.latest_rigidbodies_msg = msg

    def marker_callback(self, msg):
        self.latest_markers_msg = msg

    def update_plot(self):
        self.ax.clear()
        rb_msg = self.latest_rigidbodies_msg
        markers_msg = self.latest_markers_msg

        # Plot all markers
        if markers_msg:
            xs = [m.translation.x for m in markers_msg.markers]
            ys = [m.translation.y for m in markers_msg.markers]
            self.ax.scatter(xs, ys, marker='x', color='gray', label='all markers')
            for m in markers_msg.markers:
                self.ax.text(m.translation.x, m.translation.y, f"{m.marker_index}", fontsize=8)

        # Plot robot corners, heading, and line to goal
        if rb_msg:
            robot_body = next((rb for rb in rb_msg.rigidbodies if rb.rigid_body_name == '1'), None)
            if robot_body:
                corner_pos = []
                for i in [0,2,3,4]:
                    corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
                    if corner:
                        corner_pos.append((corner.translation.x, corner.translation.y))
                if corner_pos:
                    cp = np.array(corner_pos)
                    # robot corners
                    self.ax.scatter(cp[:,0], cp[:,1], marker='o', color='green', label='robot corners')
                    for idx, (xx, yy) in enumerate(cp):
                        self.ax.text(xx, yy, f"corner_{idx}", fontsize=8, color='green')

                    # robot center and front-center
                    x_center = np.mean(cp[:,0])
                    y_center = np.mean(cp[:,1])
                    x_front = (cp[1,0] + cp[2,0])/2.0
                    y_front = (cp[1,1] + cp[2,1])/2.0

                    # heading arrow (from center to front-center)
                    dx = x_front - x_center
                    dy = y_front - y_center
                    self.ax.arrow(x_center, y_center, dx, dy,
                                  head_width=0.02, head_length=0.03, fc='blue', ec='blue', label='heading')

                    # line from front-center to goal
                    if markers_msg:
                        goal = next((pt for pt in markers_msg.markers if pt.marker_index == GOAL_MARKER_INDEX), None)
                        if goal:
                            self.ax.plot([x_front, goal.translation.x],
                                         [y_front, goal.translation.y],
                                         'r--', label='robot-to-goal')

        # Plot formatting
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.set_title('Robot + Goal Visualization')
        self.ax.axis('equal')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = PlotRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting plotting node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
