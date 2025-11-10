#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from mocap_listener import sabertooth as st
import numpy as np
import atexit

GOAL_MARKER_INDEX = 5
MAX_ACUTUATOR_INPUT = 25
S_MAX = MAX_ACUTUATOR_INPUT * 0.6
GOAL_THRESH = 0.3

REFRESH_RATE = 10.0  # Hz
R_wheel = 0.08  # m
L = 0.178  # m
K_e = 30.0
K_theta = 50.0
ANGLE_SAT = 0.5  # radians

# Speed ramping limits
MAX_SPEED_STEP = 2.0  # max change per cycle (motor command units)

class ControllerNode(Node):
    def __init__(self, motor):
        super().__init__('controller_node')
        print("Starting GTG Controller Node")

        self.motor = motor
        atexit.register(self.motor.all_motors_off)

        self.latest_rigid = None
        self.latest_markers = None

        self.subscription_rb = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_cb,
            10
        )

        self.subscription_m = self.create_subscription(
            Markers,
            '/markers',
            self.marker_cb,
            10
        )

        self.timer = self.create_timer(1.0 / REFRESH_RATE, self.update)

        # Previous wheel speeds for ramping
        self.prev_wl = 0.0
        self.prev_wr = 0.0

    def rigid_cb(self, msg):
        self.latest_rigid = msg

    def marker_cb(self, msg):
        self.latest_markers = msg

    def update(self):
        if self.latest_rigid is None:
            return

        rb = next((r for r in self.latest_rigid.rigidbodies if r.rigid_body_name == '1'), None)
        if rb is None:
            self.get_logger().warn("Lost rigid body")
            return

        # Extract corner markers (required indices)
        corner_indices = [0, 2, 3, 4]
        pts = []
        for idx in corner_indices:
            m = next((p for p in rb.markers if p.marker_index == idx), None)
            if m is None:
                self.get_logger().warn(f"Missing marker {idx}")
                return
            pts.append(np.array([m.translation.x, m.translation.y]))

        center = np.mean(pts, axis=0)
        front = (pts[1] + pts[2]) / 2.0

        heading = front - center
        hn = np.linalg.norm(heading)
        if hn < 1e-6:
            return
        heading /= hn

        # Goal
        if self.latest_markers is None:
            return

        goal = next((p for p in self.latest_markers.markers if p.marker_index == GOAL_MARKER_INDEX), None)
        if goal is None:
            self.get_logger().warn("Missing goal marker")
            return

        goal_pos = np.array([goal.translation.x, goal.translation.y])
        dist = np.linalg.norm(center - goal_pos)

        if dist > GOAL_THRESH:
            U = K_e * (goal_pos - center)
            S_des = np.linalg.norm(U)
        else:
            S_des = 0.0

        S_des = np.clip(S_des, -S_MAX, S_MAX)

        # Angle error
        gdir = goal_pos - center
        gn = np.linalg.norm(gdir)
        if gn < 1e-6:
            gdir = heading
        else:
            gdir /= gn

        dot = heading[0] * gdir[0] + heading[1] * gdir[1]
        cross = heading[0] * gdir[1] - heading[1] * gdir[0]
        angle = np.arctan2(cross, dot)

        am = abs(angle)
        if am < ANGLE_SAT:
            w = K_theta * angle * (am / ANGLE_SAT)
        else:
            w = K_theta * angle

        wr = (S_des - L * w) / R_wheel
        wl = (S_des + L * w) / R_wheel

        wr = np.clip(wr, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        wl = np.clip(wl, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        # Speed ramping
        wl_cmd = self.prev_wl + np.clip(wl - self.prev_wl, -MAX_SPEED_STEP, MAX_SPEED_STEP)
        wr_cmd = self.prev_wr + np.clip(wr - self.prev_wr, -MAX_SPEED_STEP, MAX_SPEED_STEP)

        self.prev_wl = wl_cmd
        self.prev_wr = wr_cmd

        self.motor.updateMotorSpeed(wl_cmd, wr_cmd)

        self.get_logger().info(f"S={S_des:.2f}, angle={angle:.2f}, wl={wl_cmd:.1f}, wr={wr_cmd:.1f}")


def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True, True)
    node = ControllerNode(motor)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
    motor.all_motors_off()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
