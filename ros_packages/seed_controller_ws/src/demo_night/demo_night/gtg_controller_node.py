"""
gtg_controller_node.py

Go-To-Goal vision controller.

Changes from the original
--------------------------
* The RealSense pipeline is NO LONGER opened here.  Instead, this node
  subscribes to /vision/realsense_color (published by realsense_node.py).
  This eliminates the "device already opened" conflict with optical_path_node.
* The Arducam (V4L2) is still owned by this node — it is a separate USB device.
* All other logic (PID, pixel→robot-frame, PIVOT/DRIVE state) is unchanged.
"""
from __future__ import annotations
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from ml_red_controller.PID import PID


# ==============================================================================
# TUNING CONSTANTS
# ==============================================================================

# --- RealSense offset corrections (metres) ---
REALSENSE_OFFSET_X = -0.3
REALSENSE_OFFSET_Y =  0.16

# --- Arducam offset corrections (metres) ---
ARDUCAM_OFFSET_X = 0.1
ARDUCAM_OFFSET_Y = 0.0

# --- Camera handoff ---
# If no RealSense detection for this long, allow Arducam to take over.
REALSENSE_TIMEOUT_SEC = 0.5

# --- Goal thresholds: stop and trigger auger (metres) ---
GOAL_THRESH_REALSENSE = 0.30
GOAL_THRESH_ARDUCAM   = 0.15

# --- Motion parameters ---
MAX_ACTUATOR_INPUT = 50.0
PIVOT_THRESH       = np.deg2rad(30.0)
PIVOT_SPEED        = 40.0
DRIVE_SPEED        = 40.0

# --- PID steering (only active during DRIVE state) ---
PID_KP  = 15.0
PID_KI  = 0.5
PID_KD  = 2.0
PID_N   = 15.0
PID_KAW = 1.0

# --- HSV masking for red ---
LOWER_RED_1 = np.array([0,   190,  200])
UPPER_RED_1 = np.array([10,  255, 255])
LOWER_RED_2 = np.array([170, 190,  200])
UPPER_RED_2 = np.array([180, 255, 255])

MIN_CONTOUR_AREA = 500

# Frame staleness limit
FRAME_TIMEOUT_SEC = 0.5

# ==============================================================================


class GTGControllerNode(Node):

    def __init__(self):
        super().__init__('gtg_controller_node')

        self._load_calibration()

        # Track which camera is providing the active command.
        self.active_camera        = 'realsense'
        self.last_realsense_time  = 0   # ns: last frame where RS detected a target

        # Frame buffer for the RealSense feed (arrives via topic)
        self.bridge              = CvBridge()
        self._latest_rs_frame: np.ndarray | None = None
        self._last_rs_frame_ns: int = 0

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1.0 / 30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW,
        )

        # Publishers
        self.wheel_cmd_pub     = self.create_publisher(
            Float32MultiArray, '/vision/gtg_cmd', 10)
        self.auger_trigger_pub = self.create_publisher(
            String, '/auger/activate', 10)
        self.arc_display_pub   = self.create_publisher(
            Image, '/vision/arducam_display', 2)

        # Subscribe to RealSense frames from the central pipeline node
        self.create_subscription(
            Image, '/vision/realsense_color', self._rs_frame_cb, 2)

        # Arducam — this node still owns the V4L2 device (separate USB device)
        self.cap_arducam = cv2.VideoCapture(0)
        if self.cap_arducam.isOpened():
            self.get_logger().info("Arducam initialised.")
        else:
            self.get_logger().warn("Arducam not found or failed to open.")

        # Master vision loop at 30 Hz
        self.create_timer(1.0 / 30.0, self._process_vision)

    # -----------------------------------------------------------------------
    # Calibration
    # -----------------------------------------------------------------------

    def _load_calibration(self) -> None:
        rs_file  = (
            "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
            "ml_red_controller/ml_red_controller/calibration_data.npz"
        )
        arc_file = (
            "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
            "ml_red_controller/ml_red_controller/arducam_calibration_data.npz"
        )

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

    # -----------------------------------------------------------------------
    # RealSense frame subscription
    # -----------------------------------------------------------------------

    def _rs_frame_cb(self, msg: Image) -> None:
        self._latest_rs_frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._last_rs_frame_ns = self.get_clock().now().nanoseconds

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _reset_pid(self) -> None:
        self.pid_steer.istate     = 0.0
        self.pid_steer.dstate     = 0.0
        self.pid_steer.error_prev = 0.0

    def _pixel_to_robot_frame(
        self, cX, cY, offset_x, offset_y, pixels_per_meter, robot_x, robot_y
    ):
        dx_px   = cX - robot_x
        dy_px   = robot_y - cY
        rel_x   = (dx_px / pixels_per_meter) + offset_x
        rel_y   = (dy_px / pixels_per_meter) + offset_y
        x_robot =  rel_y
        y_robot = -rel_x
        return x_robot, y_robot

    def get_red_centroid(self, frame: np.ndarray):
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask  = cv2.bitwise_or(mask1, mask2)
        mask  = cv2.erode(mask,  None, iterations=2)
        mask  = cv2.dilate(mask, None, iterations=2)
        M     = cv2.moments(mask)
        if M['m00'] > MIN_CONTOUR_AREA:
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            return cX, cY
        return None, None

    # -----------------------------------------------------------------------
    # Main vision loop
    # -----------------------------------------------------------------------

    def _process_vision(self) -> None:
        now_ns          = self.get_clock().now().nanoseconds
        rs_target_found = False

        # -------------------------------------------------------------------
        # 1. RealSense (highest priority)
        # -------------------------------------------------------------------
        rs_frame_age_sec = (now_ns - self._last_rs_frame_ns) / 1e9
        if (self._latest_rs_frame is not None
                and rs_frame_age_sec < FRAME_TIMEOUT_SEC):

            cX, cY = self.get_red_centroid(self._latest_rs_frame)

            if cX is not None:
                rs_target_found          = True
                self.last_realsense_time = now_ns

                if self.active_camera != 'realsense':
                    self.get_logger().info(
                        "RealSense regained detection — taking control.")
                    self._reset_pid()
                    self.active_camera = 'realsense'

                x_robot, y_robot = self._pixel_to_robot_frame(
                    cX, cY,
                    REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y,
                    self.rs_pixels_per_meter, self.rs_robot_x, self.rs_robot_y,
                )
                dist = math.sqrt(x_robot ** 2 + y_robot ** 2)
                self._run_control(x_robot, y_robot, dist, GOAL_THRESH_REALSENSE)

        # -------------------------------------------------------------------
        # 2. Arducam handoff (if RealSense has lost target long enough)
        # -------------------------------------------------------------------
        realsense_age_sec = (now_ns - self.last_realsense_time) / 1e9

        if not rs_target_found and realsense_age_sec >= REALSENSE_TIMEOUT_SEC:
            if self.cap_arducam.isOpened():
                ret, frame_ard = self.cap_arducam.read()

                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(frame_ard, encoding='bgr8')
                    self.arc_display_pub.publish(img_msg)

                    cX, cY = self.get_red_centroid(frame_ard)

                    if cX is not None:
                        if self.active_camera != 'arducam':
                            self.get_logger().info(
                                f"RealSense lost for {realsense_age_sec:.1f}s "
                                "— Arducam taking control.")
                            self._reset_pid()
                            self.active_camera = 'arducam'

                        x_robot, y_robot = self._pixel_to_robot_frame(
                            cX, cY,
                            ARDUCAM_OFFSET_X, ARDUCAM_OFFSET_Y,
                            self.arc_pixels_per_meter,
                            self.arc_robot_x, self.arc_robot_y,
                        )
                        dist = math.sqrt(x_robot ** 2 + y_robot ** 2)
                        self._run_control(
                            x_robot, y_robot, dist, GOAL_THRESH_ARDUCAM)

    # -----------------------------------------------------------------------
    # Control logic
    # -----------------------------------------------------------------------

    def _run_control(
        self, x_robot: float, y_robot: float, dist: float, goal_thresh: float
    ) -> None:
        if dist <= goal_thresh:
            self.get_logger().info(
                f"Target reached (dist={dist:.2f} m)!  Triggering auger.")
            self._publish_wheels(0.0, 0.0)
            trigger_msg      = String()
            trigger_msg.data = "drill"
            self.auger_trigger_pub.publish(trigger_msg)
            return

        error_theta = math.atan2(y_robot, x_robot)

        self.get_logger().info(
            f"[{self.active_camera}] dist={dist:.2f}m  "
            f"err={np.degrees(error_theta):.1f}°  "
            f"x={x_robot:.3f}  y={y_robot:.3f}",
            throttle_duration_sec=0.5,
        )

        if abs(error_theta) > PIVOT_THRESH:
            # PIVOT: align heading before driving
            self._reset_pid()
            if error_theta > 0:
                wl, wr = -PIVOT_SPEED,  PIVOT_SPEED   # CCW
            else:
                wl, wr =  PIVOT_SPEED, -PIVOT_SPEED   # CW
        else:
            # DRIVE: PID lateral correction
            correction = self.pid_steer.update(setpoint=error_theta, output=0.0)
            wl = DRIVE_SPEED - correction
            wr = DRIVE_SPEED + correction

            max_mag = max(abs(wl), abs(wr))
            if max_mag > MAX_ACTUATOR_INPUT:
                scale = MAX_ACTUATOR_INPUT / max_mag
                wl   *= scale
                wr   *= scale

        self._publish_wheels(wl, wr)

    def _publish_wheels(self, left: float, right: float) -> None:
        msg      = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.wheel_cmd_pub.publish(msg)

    # -----------------------------------------------------------------------

    def destroy_node(self) -> None:
        if hasattr(self, 'cap_arducam') and self.cap_arducam.isOpened():
            self.cap_arducam.release()
        super().destroy_node()


# ---------------------------------------------------------------------------

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