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

REFRESH_RATE = 10  # hz
R_wheel = 0.08  # m
L = 0.178  # m
K_e = 30
K_theta = 50
ANGLE_SAT = 0.5  # radians for soft angular scaling


class ControllerNode(Node):
    def __init__(self, motor):
        print("Starting GTG Controller node")
        super().__init__('controller_node')

        # Subscribe to mocap data
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_listener_callback,
            REFRESH_RATE
        )

        self.subscription = self.create_subscription(
            Markers,
            '/markers',
            self.markers_listener_callback,
            REFRESH_RATE
        )

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
        if self.latest_rigidbodies_msg is None:
            return

        robot_body = next((rb for rb in self.latest_rigidbodies_msg.rigidbodies if rb.rigid_body_name == '1'), None)
        if robot_body is None:
            self.get_logger().info("Lost Body")
            return

        corner_pos = []
        for i in [0, 2, 3, 4]:
            corner = next((pt for pt in robot_body.markers if pt.marker_index == i), None)
            if corner is None:
                self.get_logger().warn(f"Missing corner marker {i}, dumping!")
                print(self.latest_markers_msg.markers if self.latest_markers_msg else "No markers")
                return
            corner_pos.append([corner.translation.x, corner.translation.y])

        # Compute robot center
        x_center = sum(c[0] for c in corner_pos) / len(corner_pos)
        y_center = sum(c[1] for c in corner_pos) / len(corner_pos)

        # Front of robot using markers 2 and 3
        x_front = (corner_pos[2][0] + corner_pos[3][0]) / 2
        y_front = (corner_pos[2][1] + corner_pos[3][1]) / 2

        # Heading vector (unit)
        dx = x_front - x_center
        dy = y_front - y_center
        norm = np.sqrt(dx ** 2 + dy ** 2)
        if norm > 1e-8:
            ux = dx / norm
            uy = dy / norm
        else:
            ux, uy = 0.0, 0.0

        # Goal
        x_des, y_des = 0.0, 0.0
        if self.latest_markers_msg:
            goal = next((pt for pt in self.latest_markers_msg.markers if pt.marker_index == GOAL_MARKER_INDEX), None)
            if goal is not None:
                x_des = goal.translation.x
                y_des = goal.translation.y
            else:
                self.get_logger().warn("Lost Goal")
                return
        else:
            self.get_logger().warn("No goal data")
            return

        # Control signals
        pos = robot_body.pose.position
        Ux_des = 0
        Uy_des = 0
        dist_to_goal = np.sqrt((pos.x - x_des) ** 2 + (pos.y - y_des) ** 2)
        if dist_to_goal > GOAL_THRESH:
            Ux_des = K_e * (x_des - pos.x)
            Uy_des = K_e * (y_des - pos.y)

        S_des = np.sqrt(Ux_des ** 2 + Uy_des ** 2)
        S_sat = np.clip(S_des, -S_MAX, S_MAX)

        # Robot heading unit vector (pointing forward)
        ux = -ux
        uy = -uy

        # Vector to goal
        gx = x_des - x_center
        gy = y_des - y_center
        g_norm = np.sqrt(gx ** 2 + gy ** 2)
        if g_norm > 1e-8:
            gx /= g_norm
            gy /= g_norm
        else:
            gx, gy = 0.0, 0.0

        # Angle between heading and goal
        dot = ux * gx + uy * gy
        cross = ux * gy - uy * gx
        angle = np.arctan2(cross, dot)

        # Soft scaling of angular velocity to avoid stutter
        angle_mag = abs(angle)
        if angle_mag < ANGLE_SAT:
            w_des = K_theta * angle * (angle_mag / ANGLE_SAT)
        else:
            w_des = K_theta * angle

        # Compute wheel commands
        wr_des = (S_sat - L * w_des) / R_wheel
        wl_des = (S_sat + L * w_des) / R_wheel

        # Clip wheel speeds
        wr_des_sat = np.clip(wr_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -MAX_ACUTUATOR_INPUT, MAX_ACUTUATOR_INPUT)

        # Send commands
        self.motor.updateMotorSpeed(wl_des_sat, wr_des_sat)

        # Logging
        self.get_logger().info(
            f"S_des={S_sat:.2f}, Theta_e={angle:.2f}, w_des={w_des:.2f} | Cmds L={wl_des_sat:.1f}, R={wr_des_sat:.1f}"
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
