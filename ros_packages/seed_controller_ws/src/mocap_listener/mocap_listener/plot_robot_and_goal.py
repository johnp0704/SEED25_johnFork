#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
import matplotlib.pyplot as plt
import numpy as np

GOAL_MARKER_INDEX = 5

class PlotNode(Node):
    def __init__(self):
        super().__init__("plot_robot_and_goal")

        self.latest_rigid = None
        self.latest_markers = None

        self.create_subscription(RigidBodies, '/rigid_bodies', self.rb_cb, 10)
        self.create_subscription(Markers, '/markers', self.markers_cb, 10)

        self.timer = self.create_timer(0.1, self.update_plot)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.robot_plot, = self.ax.plot([], [], 'bo-', linewidth=2)
        self.goal_plot, = self.ax.plot([], [], 'rx', markersize=12)

    def rb_cb(self, msg):
        self.latest_rigid = msg

    def markers_cb(self, msg):
        self.latest_markers = msg

    def update_plot(self):
        if self.latest_rigid is None or self.latest_markers is None:
            return

        rb = next((r for r in self.latest_rigid.rigidbodies if r.rigid_body_name == '1'), None)
        if rb is None:
            return

        pts = []
        for idx in [0, 2, 3, 4]:
            m = next((p for p in rb.markers if p.marker_index == idx), None)
            if m is None:
                return
            pts.append([m.translation.x, m.translation.y])

        pts = np.array(pts)
        self.robot_plot.set_data(pts[:,0], pts[:,1])

        goal = next((p for p in self.latest_markers.markers if p.marker_index == GOAL_MARKER_INDEX), None)
        if goal:
            self.goal_plot.set_data(goal.translation.x, goal.translation.y)

        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        plt.draw()
        plt.pause(0.001)


def main():
    rclpy.init()
    node = PlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
