#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
from sabertooth import SaberToothMotorDriver


class GoToGoalController(Node):
    def __init__(self):
        super().__init__('go_to_goal_controller')

        # Controller gains (static, no live tuning)
        self.K_e = 0.6       # Linear error gain
        self.K_theta = 2.0   # Heading gain

        # Speed scaling for Sabertooth (-100 to +100)
        self.MAX_OUTPUT = 80.0   # Leave headroom (avoid instantly slamming motors)
        self.MAX_SPEED_STEP = 10.0  # Ramp rate per update

        # Robot specs
        self.wheel_base = 0.24  # meters between wheels

        # Initialize motor driver
        self.motor = SaberToothMotorDriver(True, True)  # (ports reversed?, debug?)

        # State
        self.x = None
        self.y = None
        self.yaw = None
        self.goal = np.array([0.0, 0.0])

        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0

        # Subscribers
        self.create_subscription(Float32MultiArray, "/rigid_bodies", self.rigid_body_callback, 10)
        self.create_subscription(PoseStamped, "/goal", self.goal_callback, 10)

        # Timer loop (50 Hz)
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Go-To-Goal controller started.")


    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])


    def rigid_body_callback(self, msg):
        # Expected msg: [x, y, z, qx, qy, qz, qw]
        data = msg.data
        if len(data) < 7:
            return

        self.x = data[0]
        self.y = data[1]

        # Convert quaternion to yaw
        qx, qy, qz, qw = data[3:7]
        # yaw from quaternion
        self.yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))


    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        e = np.hypot(dx, dy)  # distance to goal

        # Heading angle to the goal
        theta_g = np.arctan2(dy, dx)

        # Heading error
        theta_error = theta_g - self.yaw
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))  # wrap

        # Control law
        v = self.K_e * e
        w = self.K_theta * theta_error

        # Convert to wheel speeds
        wl = v - (w * self.wheel_base / 2.0)
        wr = v + (w * self.wheel_base / 2.0)

        # Scale to sabertooth input
        wl = np.clip(wl * 20.0, -self.MAX_OUTPUT, self.MAX_OUTPUT)
        wr = np.clip(wr * 20.0, -self.MAX_OUTPUT, self.MAX_OUTPUT)

        # Speed ramping
        wl = self.last_left_cmd + np.clip(wl - self.last_left_cmd, -self.MAX_SPEED_STEP, self.MAX_SPEED_STEP)
        wr = self.last_right_cmd + np.clip(wr - self.last_right_cmd, -self.MAX_SPEED_STEP, self.MAX_SPEED_STEP)

        # Store for next step
        self.last_left_cmd = wl
        self.last_right_cmd = wr

        # Send to motors (expects -100 to 100)
        self.motor.updateMotorSpeed(float(wl), float(wr))


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
