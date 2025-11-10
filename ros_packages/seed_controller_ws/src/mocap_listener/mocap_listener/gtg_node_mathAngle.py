#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies, Markers
from scipy.spatial.transform import Rotation as R
from mocap_listener import sabertooth as st
import atexit
import numpy as np

GOAL_MARKER_INDEX = 5
MAX_ACUTUATOR_INPUT = 25
S_MAX = MAX_ACUTUATOR_INPUT * 0.6
GOAL_THRESH = 0.3

REFRESH_RATE = 10  # Hz
R_wheel = 0.08  # m
L = 0.178  # m
K_e = 30
K_theta = 50
ANGLE_THRESHOLD = 0.05  # radians (~3 degrees)
SLOWDOWN_RADIUS = 0.5   # meters


class ControllerNode(Node):
    def __init__(self, motor):
        print("Starting GTG Controller node")
        super().__init__('controller_node')

        # Subscribe to mocap data
        self.create_subscription(RigidBodies, '/rigid_bodies',
                                 self.rigid_bodies_listener_callback, REFRESH_RATE)
        self.create_subscription(Markers, '/markers',
                                 self.markers_listener_callback, REFRESH_RATE)

        self.motor = motor
        atexit.register(self.motor.all_motors_off)

        self.timer = self.create_timer(1 / REFRESH_RATE, self.controller_update)
        self.latest_rigidbodies_msg = None
        self.latest_markers_msg = None

    def rigid_bodies_listener_callback(self, msg: RigidBodies):
        self.latest_rigidbodies_msg = msg

    def markers_listener_callback(self, msg: Markers):
        self.latest_markers_msg = msg

    def controller_update(self):
        # Get robot pose
        if self.latest_rigidbodies_msg is not None:
            robot_body = next((rb for rb in self.latest_rigidbodies_msg.rigidbodies
                               if rb.rigid_body_name == '1'), None)
        else:
            robot_body = None

        if robot_body is None or self.latest_markers_msg is None:
            return

        # --- Compute robot heading from front and back corner markers ---
        front_markers = [m for m in robot_body.markers if m.marker_index in [2, 3]]
        back_markers = [m for m in robot_body.markers if m.marker_index in [0, 4]]

        if len(front_markers) < 2 or len(back_markers) < 2:
            self.get_logger().warn("Missing front/back markers for heading computation")
            return

        # Midpoints of front and back edges
        front_x = np.mean([m.translation.x for m in front_markers])
        front_y = np.mean([m.translation.y for m in front_markers])
        back_x = np.mean([m.translation.x for m in back_markers])
        back_y = np.mean([m.translation.y for m in back_markers])

        # Heading vector points from back → front
        dx = front_x - back_x
        dy = front_y - back_y
        norm = np.sqrt(dx ** 2 + dy ** 2)
        if norm > 1e-8:
            ux = dx / norm
            uy = dy / norm
        else:
            ux, uy = 0.0, 0.0

        # Robot center
        x_center = np.mean([m.translation.x for m in front_markers + back_markers])
        y_center = np.mean([m.translation.y for m in front_markers + back_markers])

        # Goal
        goal = next((pt for pt in self.latest_markers_msg.markers
                     if pt.marker_index == GOAL_MARKER_INDEX), None)
        if goal is None:
            self.get_logger().warn("Lost Goal")
            return

        x_des = goal.translation.x
        y_des = goal.translation.y

        # Distance to goal
        dist_to_goal = np.sqrt((x_center - x_des) ** 2 + (y_center - y_des) ** 2)

        # --- Compute linear velocity ---
        if dist_to_goal > GOAL_THRESH:
            Ux_des = K_e * (x_des - x_center)
            Uy_des = K_e * (y_des - y_center)
        else:
            Ux_des = 0.0
            Uy_des = 0.0

        S_des = np.sqrt(Ux_des ** 2 + Uy_des ** 2)
        S_sat = np.clip(S_des, -S_MAX, S_MAX)

        # Slow down near goal
        if dist_to_goal < SLOWDOWN_RADIUS:
            scale = dist_to_goal / SLOWDOWN_RADIUS
            S_sat *= scale
            K_theta_scaled = K_theta * scale
        else:
            K_theta_scaled = K_theta

        # --- Compute angular velocity ---
        # Vector to goal
        gx = x_des - x_center
        gy = y_des - y_center
        g_norm = np.sqrt(gx ** 2 + gy ** 2)
        if g_norm > 1e-8:
            gx /= g_norm
            gy /= g_norm
        else:
            gx, gy = 0.0, 0.0

        # Signed angle between heading and goal vector
        dot = ux * gx + uy * gy
        cross = ux * gy - uy * gx
        angle = np.arctan2(cross, dot)

        # Apply angle threshold to prevent jitter near goal
        if abs(angle) < ANGLE_THRESHOLD:
            w_des = 0.0
        else:
            w_des = K_theta_scaled * angle

        # Differential drive wheel velocities
        wr_des = (S_sat - L * w_des) / R_wheel
        wl_des = (S_sat + L * w_des) / R_wheel

        # Saturate motor commands
        wr_des_sat = np.clip(wr_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        # Send commands to motors
        self.motor.updateMotorSpeed(wl_des_sat, wr_des_sat)

        # Debug log
        self.get_logger().info(
            f"S_des={S_sat:.2f}, Theta_e={angle:.2f}, w_des={w_des:.2f} | "
            f"Cmds L={wl_des_sat:.1f}, R={wr_des_sat:.1f} | Heading=({ux:.2f},{uy:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    motor = st.SaberToothMotorDriver(True, True)
    node = ControllerNode(motor)
    atexit.register(motor.all_motors_off)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
        motor.all_motors_off()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
