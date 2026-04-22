import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import math
import os
from ml_red_controller.PID import PID


# ==============================================================================
# TUNING CONSTANTS
# ==============================================================================

# --- RealSense offset corrections (meters) ---
# Positive OFFSET_X moves the effective goal further forward (robot stops further away)
# Positive OFFSET_Y moves the effective goal to the left
REALSENSE_OFFSET_X = -0.3
REALSENSE_OFFSET_Y =  0.16

# --- Arducam offset corrections (meters) ---
ARDUCAM_OFFSET_X = 0.1
ARDUCAM_OFFSET_Y = 0.0

# --- Camera handoff ---
# RealSense hands off to Arducam when it loses detection for this long
REALSENSE_TIMEOUT_SEC = 0.5

# --- Goal threshold: stop and trigger auger within this distance (meters) ---
GOAL_THRESH_REALSENSE = 0.30
GOAL_THRESH_ARDUCAM   = 0.15

# --- Motion parameters ---
MAX_ACTUATOR_INPUT = 50.0
PIVOT_THRESH       = np.deg2rad(30.0)
PIVOT_SPEED        = 40.0
DRIVE_SPEED        = 40.0
COOLDOWN_DUR       = 15.0

# --- PID steering (only active during DRIVE state) ---
PID_KP  = 15.0
PID_KI  = 0.5
PID_KD  = 2.0
PID_N   = 15.0
PID_KAW = 1.0

# ==============================================================================


class GTGControllerNode(Node):
    def __init__(self):
        super().__init__('gtg_controller_node')

        self.load_calibration()

        self.cooldown_end_time   = 0
        self.active_camera       = 'realsense'
        self.last_realsense_time = 0

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1/30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW
        )

        self.wheel_cmd_pub     = self.create_publisher(Float32MultiArray, '/vision/wheel_cmd', 10)
        self.auger_trigger_pub = self.create_publisher(String, '/auger/activate', 10)

        self.create_subscription(Point, '/vision/realsense_target', self.realsense_callback, 10)
        self.create_subscription(Point, '/vision/arducam_target',   self.arducam_callback,   10)

    def load_calibration(self):
        rs_file  = "/home/airlab/seed25/ros_packages/seed_controller_ws/src/ml_red_controller/ml_red_controller/calibration_data.npz"
        arc_file = "/home/airlab/seed25/ros_packages/seed_controller_ws/src/ml_red_controller/ml_red_controller/arducam_calibration_data.npz"

        if os.path.exists(rs_file):
            data = np.load(rs_file)
            self.rs_pixels_per_meter = float(data['pixels_per_meter'])
            self.rs_robot_x          = int(data['robot_x'])
            self.rs_robot_y          = int(data['robot_y'])
            self.get_logger().info("RealSense calibration loaded.")
        else:
            self.get_logger().error(f"RealSense calibration not found: {rs_file}")
            self.rs_pixels_per_meter = 1.0
            self.rs_robot_x = 0
            self.rs_robot_y = 0

        if os.path.exists(arc_file):
            data = np.load(arc_file)
            self.arc_pixels_per_meter = float(data['pixels_per_meter'])
            self.arc_robot_x          = int(data['robot_x'])
            self.arc_robot_y          = int(data['robot_y'])
            self.get_logger().info("Arducam calibration loaded.")
        else:
            self.get_logger().warn(f"Arducam calibration not found: {arc_file}")
            self.arc_pixels_per_meter = 1.0
            self.arc_robot_x = 0
            self.arc_robot_y = 0

    def _reset_pid(self):
        self.pid_steer.istate     = 0.0
        self.pid_steer.dstate     = 0.0
        self.pid_steer.error_prev = 0.0

    def _pixel_to_robot_frame(self, cX, cY, offset_x, offset_y, pixels_per_meter, robot_x, robot_y):
        dx_px   = cX - robot_x
        dy_px   = robot_y - cY
        rel_x   = (dx_px / pixels_per_meter) + offset_x
        rel_y   = (dy_px / pixels_per_meter) + offset_y
        x_robot =  rel_y
        y_robot = -rel_x
        return x_robot, y_robot

    def realsense_callback(self, msg):
        self.last_realsense_time = self.get_clock().now().nanoseconds
        if self.active_camera != 'realsense':
            self.get_logger().info("RealSense regained detection — taking control.")
            self._reset_pid()
            self.active_camera = 'realsense'
        x_robot, y_robot = self._pixel_to_robot_frame(
            msg.x, msg.y,
            REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y,
            self.rs_pixels_per_meter, self.rs_robot_x, self.rs_robot_y
        )
        dist = math.sqrt(x_robot**2 + y_robot**2)
        self._run_control(x_robot, y_robot, dist, GOAL_THRESH_REALSENSE)

    def arducam_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        realsense_age_sec = (now_ns - self.last_realsense_time) / 1e9
        if realsense_age_sec < REALSENSE_TIMEOUT_SEC:
            return
        if self.active_camera != 'arducam':
            self.get_logger().info(
                f"RealSense lost for {realsense_age_sec:.1f}s — Arducam taking control."
            )
            self._reset_pid()
            self.active_camera = 'arducam'
        x_robot, y_robot = self._pixel_to_robot_frame(
            msg.x, msg.y,
            ARDUCAM_OFFSET_X, ARDUCAM_OFFSET_Y,
            self.arc_pixels_per_meter, self.arc_robot_x, self.arc_robot_y
        )
        dist = math.sqrt(x_robot**2 + y_robot**2)
        self._run_control(x_robot, y_robot, dist, GOAL_THRESH_ARDUCAM)

    def _run_control(self, x_robot, y_robot, dist, goal_thresh):
        # Cooldown check
        if self.get_clock().now().nanoseconds < self.cooldown_end_time:
            return

        # Goal reached — stop and trigger auger
        if dist <= goal_thresh:
            self.get_logger().info(f"Target reached (dist={dist:.2f}m)! Triggering auger.")
            self._publish_wheels(0.0, 0.0)
            trigger_msg      = String()
            trigger_msg.data = "drill"
            self.auger_trigger_pub.publish(trigger_msg)
            self.cooldown_end_time = (
                self.get_clock().now().nanoseconds + int(COOLDOWN_DUR * 1e9)
            )
            return

        error_theta = math.atan2(y_robot, x_robot)

        # Diagnostic log — shows sign of error vs physical target position
        self.get_logger().info(
            f"[{self.active_camera}] dist={dist:.2f}m  "
            f"err={np.degrees(error_theta):.1f}deg  "
            f"x_robot={x_robot:.3f} y_robot={y_robot:.3f}",
            throttle_duration_sec=0.5
        )

        # ----------------------------------------------------------
        # PIVOT: full counter-rotation until heading is aligned
        # Verified convention (True, True wiring):
        #   Turn left  CCW = (-X, +X)
        #   Turn right CW  = (+X, -X)
        # error_theta > 0 = target is to the LEFT
        # error_theta < 0 = target is to the RIGHT
        # ----------------------------------------------------------
        if abs(error_theta) > PIVOT_THRESH:
            self._reset_pid()
            if error_theta > 0:
                wl_des = -PIVOT_SPEED
                wr_des =  PIVOT_SPEED
            else:
                wl_des =  PIVOT_SPEED
                wr_des = -PIVOT_SPEED

        # ----------------------------------------------------------
        # DRIVE: heading aligned, PID correction mixed into wheels
        # ----------------------------------------------------------
        else:
            correction = self.pid_steer.update(setpoint=error_theta, output=0.0)
            wl_des = DRIVE_SPEED - correction
            wr_des = DRIVE_SPEED + correction

            max_input = max(abs(wl_des), abs(wr_des))
            if max_input > MAX_ACTUATOR_INPUT:
                scale  = MAX_ACTUATOR_INPUT / max_input
                wl_des *= scale
                wr_des *= scale

        self._publish_wheels(wl_des, wr_des)

    def _publish_wheels(self, left, right):
        msg      = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.wheel_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GTGControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()