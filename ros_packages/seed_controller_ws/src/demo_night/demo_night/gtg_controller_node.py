"""
gtg_controller_node.py

Go-To-Goal vision controller.

Camera priority (revised)
--------------------------
The Arducam is mounted lower and closer to the ground — it keeps the weed
in frame longer as the robot approaches and is more reliable for the final
approach.  The RealSense has a wider FOV and is used as a long-range scout.

Priority order each tick:
  1. Arducam  — if it has a live red detection, it drives.
  2. RealSense — if Arducam sees nothing but RealSense does, RealSense drives.
  3. Neither  — publish nothing; commander holds the robot still (GTG mode)
                or falls back to optical (OPTICAL mode).

Separate HSV thresholds
-----------------------
The Arducam produces a noticeably different colour response under indoor
lighting — the red mat appears more desaturated/darker than on the RealSense.
ARC_LOWER/UPPER_RED_* use looser S and V minimums to catch it.

If you get false positives with the Arducam, raise ARC_S_MIN back toward 80
in steps of 10 until they go away.

Cooldown integration
--------------------
After a successful drill cycle the commander publishes a float string on
/gtg/cooldown (e.g. "10.0").  During that window this node will NOT
publish wheel commands or auger triggers.
"""
from __future__ import annotations
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

import cv2
import numpy as np
from ml_red_controller.PID import PID

# ==============================================================================
# TUNING CONSTANTS
# ==============================================================================

REALSENSE_OFFSET_X    = -0.3
REALSENSE_OFFSET_Y    =  0.16
ARDUCAM_OFFSET_X      =  -0.05
ARDUCAM_OFFSET_Y      =  0.0

# "Is the camera alive?" check only — does NOT gate detection handoff.
FRAME_TIMEOUT_SEC = 0.5

# Stopping distances.
GOAL_THRESH_REALSENSE =  0.20
GOAL_THRESH_ARDUCAM   =  0.18

MAX_ACTUATOR_INPUT    = 50.0
PIVOT_THRESH          = np.deg2rad(30.0)
PIVOT_SPEED           = 40.0
DRIVE_SPEED           = 40.0
PID_KP  = 15.0
PID_KI  =  0.5
PID_KD  =  2.0
PID_N   = 15.0
PID_KAW =  1.0

# ------------------------------------------------------------------------------
# RealSense HSV thresholds — conservative; the RealSense pipeline is
# well white-balanced so red is vivid.
# ------------------------------------------------------------------------------
RS_LOWER_RED_1 = np.array([0,   120, 120])
RS_UPPER_RED_1 = np.array([15,  255, 255])
RS_LOWER_RED_2 = np.array([165, 120, 120])
RS_UPPER_RED_2 = np.array([180, 255, 255])
RS_MIN_AREA    = 300

# ------------------------------------------------------------------------------
# Arducam HSV thresholds — looser S and V floors.
# Under indoor demo lighting the red mat looks desaturated/darker on the
# Arducam sensor.  The hue band is also slightly wider for lens colour shift.
#
# TUNING: if you see false positives on walls/floor, raise ARC_S_MIN toward
# 80 in steps of 10.  If the target is still missed, lower it toward 40.
# ------------------------------------------------------------------------------
ARC_S_MIN = 60    # was 120 on RealSense — key relaxation
ARC_V_MIN = 60    # was 120 on RealSense

ARC_LOWER_RED_1 = np.array([0,   ARC_S_MIN, ARC_V_MIN])
ARC_UPPER_RED_1 = np.array([18,  255,        255       ])
ARC_LOWER_RED_2 = np.array([162, ARC_S_MIN, ARC_V_MIN])
ARC_UPPER_RED_2 = np.array([180, 255,        255       ])
ARC_MIN_AREA    = 150   # lower bar — target is smaller/more washed-out

# ==============================================================================


class GTGControllerNode(Node):

    def __init__(self):
        super().__init__('gtg_controller_node')

        self._load_calibration()

        self.bridge = CvBridge()

        self._latest_rs_frame:   np.ndarray | None = None
        self._last_rs_frame_ns:  int = 0
        self._latest_arc_frame:  np.ndarray | None = None
        self._last_arc_frame_ns: int = 0

        self._cooldown_until:  float = 0.0
        self._auger_triggered: bool  = False

        self.active_camera: str = 'none'

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1.0 / 30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW,
        )

        self.wheel_cmd_pub      = self.create_publisher(
            Float32MultiArray, '/vision/gtg_cmd', 10)
        self.auger_trigger_pub  = self.create_publisher(
            String, '/auger/activate', 10)
        # Separate debug mask topics so you can inspect each camera in RViz.
        self.rs_debug_mask_pub  = self.create_publisher(
            Image, '/vision/gtg_debug_mask',     SENSOR_QOS)
        self.arc_debug_mask_pub = self.create_publisher(
            Image, '/vision/gtg_arc_debug_mask', SENSOR_QOS)

        self.create_subscription(
            Image, '/vision/realsense_color', self._rs_frame_cb, SENSOR_QOS)
        self.create_subscription(
            Image, '/vision/arducam_raw', self._arc_frame_cb, SENSOR_QOS)
        self.create_subscription(
            String, '/gtg/cooldown', self._cooldown_cb, 10)

        self.create_timer(1.0 / 30.0, self._process_vision)
        self.get_logger().info(
            "GTG controller initialised.  "
            "Priority: Arducam (close-range finisher) → RealSense (wide-FOV scout).")

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
            self.rs_robot_x = self.rs_robot_y = 0

        if os.path.exists(arc_file):
            data = np.load(arc_file)
            self.arc_pixels_per_meter = float(data['pixels_per_meter'])
            self.arc_robot_x          = int(data['robot_x'])
            self.arc_robot_y          = int(data['robot_y'])
            self.get_logger().info("Arducam calibration loaded.")
        else:
            self.get_logger().warn(f"Arducam calibration not found: {arc_file}")
            self.arc_pixels_per_meter = 1.0
            self.arc_robot_x = self.arc_robot_y = 0

    # -----------------------------------------------------------------------
    # Subscriptions
    # -----------------------------------------------------------------------

    def _cooldown_cb(self, msg: String) -> None:
        try:
            duration = float(msg.data)
        except ValueError:
            self.get_logger().warn(
                f"[GTG] Invalid cooldown value '{msg.data}' — ignored.")
            return
        self._cooldown_until  = time.monotonic() + duration
        self._auger_triggered = False
        self.get_logger().info(f"[GTG] Cooldown active for {duration:.1f}s.")

    def _in_cooldown(self) -> bool:
        return time.monotonic() < self._cooldown_until

    def _rs_frame_cb(self, msg: Image) -> None:
        self._latest_rs_frame  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self._last_rs_frame_ns = self.get_clock().now().nanoseconds

    def _arc_frame_cb(self, msg: Image) -> None:
        self._latest_arc_frame  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self._last_arc_frame_ns = self.get_clock().now().nanoseconds

    def _reset_pid(self) -> None:
        self.pid_steer.istate = \
            self.pid_steer.dstate = \
            self.pid_steer.error_prev = 0.0

    # -----------------------------------------------------------------------
    # Detection — separate tuning per camera
    # -----------------------------------------------------------------------

    def _pixel_to_robot_frame(self, cX, cY, offset_x, offset_y,
                               pixels_per_meter, robot_x, robot_y):
        dx_px = cX - robot_x
        dy_px = robot_y - cY
        rel_x = (dx_px / pixels_per_meter) + offset_x
        rel_y = (dy_px / pixels_per_meter) + offset_y
        return rel_y, -rel_x

    def _get_red_centroid_realsense(self, frame: np.ndarray):
        """Red detection tuned for the RealSense colour pipeline."""
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RS_LOWER_RED_1, RS_UPPER_RED_1),
            cv2.inRange(hsv, RS_LOWER_RED_2, RS_UPPER_RED_2),
        )
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        try:
            self.rs_debug_mask_pub.publish(
                self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))
        except Exception:
            pass
        M = cv2.moments(mask)
        if M['m00'] > RS_MIN_AREA:
            return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        return None, None

    def _get_red_centroid_arducam(self, frame: np.ndarray):
        """
        Red detection tuned for the Arducam under indoor demo lighting.

        Key differences from RealSense:
          - Lower S and V minimums — the target looks desaturated/darker.
          - Slightly wider hue band for lens colour shift.
          - Only 1 erosion pass — prevents killing small/low-contrast detections.
          - Lower minimum area threshold.
        """
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, ARC_LOWER_RED_1, ARC_UPPER_RED_1),
            cv2.inRange(hsv, ARC_LOWER_RED_2, ARC_UPPER_RED_2),
        )
        mask = cv2.erode(mask,  None, iterations=1)   # gentler than RS
        mask = cv2.dilate(mask, None, iterations=2)
        try:
            self.arc_debug_mask_pub.publish(
                self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))
        except Exception:
            pass
        M = cv2.moments(mask)
        if M['m00'] > ARC_MIN_AREA:
            return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        return None, None

    # -----------------------------------------------------------------------
    # Main vision loop
    # -----------------------------------------------------------------------

    def _process_vision(self) -> None:
        if self._in_cooldown():
            remaining = self._cooldown_until - time.monotonic()
            self.get_logger().info(
                f"[GTG] In cooldown — {remaining:.1f}s remaining.",
                throttle_duration_sec=2.0)
            return

        now_ns = self.get_clock().now().nanoseconds

        arc_frame_age = (now_ns - self._last_arc_frame_ns) / 1e9
        arc_alive     = (self._latest_arc_frame is not None
                         and arc_frame_age < FRAME_TIMEOUT_SEC)

        rs_frame_age  = (now_ns - self._last_rs_frame_ns) / 1e9
        rs_alive      = (self._latest_rs_frame is not None
                         and rs_frame_age < FRAME_TIMEOUT_SEC)

        # ------------------------------------------------------------------
        # 1. Arducam — preferred; close-range finisher.
        # ------------------------------------------------------------------
        if arc_alive:
            cX, cY = self._get_red_centroid_arducam(self._latest_arc_frame)

            if cX is not None:
                if self.active_camera != 'arducam':
                    self.get_logger().info(
                        "Arducam has detection — taking control.")
                    self._reset_pid()
                    self.active_camera = 'arducam'

                x_robot, y_robot = self._pixel_to_robot_frame(
                    cX, cY, ARDUCAM_OFFSET_X, ARDUCAM_OFFSET_Y,
                    self.arc_pixels_per_meter, self.arc_robot_x, self.arc_robot_y)
                dist = math.sqrt(x_robot**2 + y_robot**2)

                self.get_logger().info(
                    f"[GTG ARC] centroid=({cX},{cY}) dist={dist:.2f}m",
                    throttle_duration_sec=0.5)

                self._run_control(x_robot, y_robot, dist, GOAL_THRESH_ARDUCAM)
                return

            else:
                if self.active_camera == 'arducam':
                    self.get_logger().info(
                        "[GTG ARC] Lost centroid — falling back to RealSense.")
                    self.active_camera = 'none'

        # ------------------------------------------------------------------
        # 2. RealSense — wide-FOV scout when Arducam has no detection.
        # ------------------------------------------------------------------
        if rs_alive:
            cX, cY = self._get_red_centroid_realsense(self._latest_rs_frame)

            if cX is not None:
                if self.active_camera != 'realsense':
                    self.get_logger().info(
                        "RealSense has detection — taking control.")
                    self._reset_pid()
                    self.active_camera = 'realsense'

                x_robot, y_robot = self._pixel_to_robot_frame(
                    cX, cY, REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y,
                    self.rs_pixels_per_meter, self.rs_robot_x, self.rs_robot_y)
                dist = math.sqrt(x_robot**2 + y_robot**2)

                self.get_logger().info(
                    f"[GTG RS] centroid=({cX},{cY}) dist={dist:.2f}m",
                    throttle_duration_sec=0.5)

                self._run_control(x_robot, y_robot, dist, GOAL_THRESH_REALSENSE)
                return

            else:
                if self.active_camera == 'realsense':
                    self.get_logger().info(
                        "[GTG RS] Lost centroid — no camera has detection.")
                    self.active_camera = 'none'

        # ------------------------------------------------------------------
        # 3. No detection on either camera — hold still.
        # ------------------------------------------------------------------
        if self.active_camera != 'none':
            self.active_camera = 'none'
        self.get_logger().info(
            "[GTG] No detection on either camera — waiting.",
            throttle_duration_sec=2.0)

    # -----------------------------------------------------------------------
    # Control
    # -----------------------------------------------------------------------

    def _run_control(self, x_robot, y_robot, dist, goal_thresh) -> None:
        if dist <= goal_thresh:
            if not self._auger_triggered:
                self._auger_triggered = True
                self.get_logger().info(
                    f"Target reached (dist={dist:.2f}m ≤ {goal_thresh}m). "
                    "Triggering auger (one-shot).")
                self._publish_wheels(0.0, 0.0)
                msg = String(); msg.data = "drill"
                self.auger_trigger_pub.publish(msg)
            return

        error_theta = math.atan2(y_robot, x_robot)
        self.get_logger().info(
            f"[{self.active_camera}] dist={dist:.2f}m  "
            f"err={np.degrees(error_theta):.1f}°",
            throttle_duration_sec=0.5)

        if abs(error_theta) > PIVOT_THRESH:
            self._reset_pid()
            wl, wr = (-PIVOT_SPEED, PIVOT_SPEED) if error_theta > 0 \
                     else (PIVOT_SPEED, -PIVOT_SPEED)
        else:
            correction = self.pid_steer.update(setpoint=error_theta, output=0.0)
            wl = DRIVE_SPEED - correction
            wr = DRIVE_SPEED + correction
            mag = max(abs(wl), abs(wr))
            if mag > MAX_ACTUATOR_INPUT:
                wl *= MAX_ACTUATOR_INPUT / mag
                wr *= MAX_ACTUATOR_INPUT / mag

        self._publish_wheels(wl, wr)

    def _publish_wheels(self, left, right) -> None:
        msg = Float32MultiArray()
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