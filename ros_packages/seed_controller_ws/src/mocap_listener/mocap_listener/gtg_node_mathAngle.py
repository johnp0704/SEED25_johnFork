#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
import atexit
import numpy as np
from mocap_listener import sabertooth as st

# Marker configuration
GOAL_MARKER_INDEX = 5
CORNER_MARKERS = [0, 2, 3, 4]

# Robot geometry
R_WHEEL = 0.08
L = 0.178

# Control gains
K_v = 25.0          # Linear gain
K_theta = 40.0      # Angular gain
ANGLE_FORWARD_THRESH = 0.6   # rad, ~34 deg where robot is allowed to drive forward

# Speed limits
MAX_CMD = 25
MAX_ACCEL = 2.0      # max actuator change per update step

# Goal behavior
GOAL_THRESH = 0.30

# Loop frequency
REFRESH_RATE = 20.0


class ControllerNode(Node):
    def __init__(self, motor):
        super().__init__("controller_node")
        print("Starting GTG Controller")

        self.motor = motor
        atexit.register(self.motor.all_motors_off)

        self.latest_rb = None
        self.latest_markers = None

        self.last_left = 0.0
        self.last_right = 0.0

        self.create_subscription(RigidBodies, "/rigid_bodies", self.rb_cb, 10)
        self.create_subscription(Markers, "/markers", self.mark_cb, 10)

        self.timer = self.create_timer(1.0 / REFRESH_RATE, self.update)

    def rb_cb(self, msg):
        self.latest_rb = msg

    def mark_cb(self, msg):
        self.latest_markers = msg

    def update(self):
        if self.latest_rb is None or self.latest_markers is None:
            return

        # Locate robot body
        rb = next((r for r in self.latest_rb.rigidbodies if r.rigid_body_name == "1"), None)
        if rb is None:
            self.get_logger().warn("Lost robot body")
            return

        # Extract corner markers
        corners = []
        for idx in CORNER_MARKERS:
            m = next((p for p in rb.markers if p.marker_index == idx), None)
            if m is None:
                self.get_logger().warn(f"Missing robot marker {idx}")
                return
            corners.append(np.array([m.translation.x, m.translation.y]))

        center = np.mean(corners, axis=0)
        front = (corners[1] + corners[2]) / 2.0
        heading = front - center
        norm = np.linalg.norm(heading)
        if norm < 1e-8:
            return
        ux, uy = heading / norm

        # Get goal marker
        goal = next((m for m in self.latest_markers.markers if m.marker_index == GOAL_MARKER_INDEX), None)
        if goal is None:
            self.get_logger().warn("Lost goal marker")
            return

        xg, yg = goal.translation.x, goal.translation.y
        dx = xg - center[0]
        dy = yg - center[1]

        dist = np.hypot(dx, dy)

        # If close, stop smoothly
        if dist < GOAL_THRESH:
            self.ramped_send(0.0, 0.0)
            return

        # Direction to goal
        gn = np.hypot(dx, dy)
        gx, gy = dx / gn, dy / gn

        # Angle error
        dot = ux * gx + uy * gy
        cross = ux * gy - uy * gx
        ang = np.arctan2(cross, dot)

        # Only move forward if facing roughly correct direction
        if abs(ang) < ANGLE_FORWARD_THRESH:
            v = K_v * dist
        else:
            v = 0.0

        w = K_theta * ang

        # Convert to wheel speeds
        wr = (v - L * w) / R_WHEEL
        wl = (v + L * w) / R_WHEEL

        # Saturate
        wr = np.clip(wr, -MAX_CMD, MAX_CMD)
        wl = np.clip(wl, -MAX_CMD, MAX_CMD)

        # Smooth command (ramping)
        self.ramped_send(wl, wr)

        self.get_logger().info(f"dist={dist:.2f}, ang={ang:.2f} | L={self.last_left:.1f}, R={self.last_right:.1f}")

    def ramped_send(self, wl, wr):
        wl_cmd = self.last_left + np.clip(wl - self.last_left, -MAX_ACCEL, MAX_ACCEL)
        wr_cmd = self.last_right + np.clip(wr - self.last_right, -MAX_ACCEL, MAX_ACCEL)

        self.last_left = wl_cmd
        self.last_right = wr_cmd

        self.motor.updateMotorSpeed(wl_cmd, wr_cmd)


def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True, True)
    node = ControllerNode(motor)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    motor.all_motors_off()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
