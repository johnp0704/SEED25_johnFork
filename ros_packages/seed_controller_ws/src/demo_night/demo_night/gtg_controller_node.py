"""
gtg_controller_node.py

Go-To-Goal vision controller.

Key addition: publishes a /vision/gtg_debug_mask topic (sensor_msgs/Image)
showing the combined red HSV mask.  Subscribe to this with:
    ros2 run rqt_image_view rqt_image_view /vision/gtg_debug_mask
to confirm whether the red target is being detected at all.

If the mask shows nothing when pointing at the red target, the HSV ranges
need updating with the realsense_color_sampler script.
"""
from __future__ import annotations
import math
import os

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
ARDUCAM_OFFSET_X      =  0.1
ARDUCAM_OFFSET_Y      =  0.0
REALSENSE_TIMEOUT_SEC =  0.5
GOAL_THRESH_REALSENSE =  0.30
GOAL_THRESH_ARDUCAM   =  0.15
MAX_ACTUATOR_INPUT    = 50.0
PIVOT_THRESH          = np.deg2rad(30.0)
PIVOT_SPEED           = 40.0
DRIVE_SPEED           = 40.0
PID_KP  = 15.0
PID_KI  =  0.5
PID_KD  =  2.0
PID_N   = 15.0
PID_KAW =  1.0

# HSV red mask — two ranges because red wraps around hue=0/180.
#
# Widened from the original sampler-derived values to be more robust to
# lighting variation:
#   • Hue ranges extended by ±2 ticks on both sides
#   • Saturation minimum lowered from 190 → 140  (catches less-saturated reds)
#   • Value minimum lowered from 200 → 140        (catches darker reds / shadows)
#
# If you still get false positives (noise), raise the minimums back up
# in small steps.  If you get false negatives, lower them further.
LOWER_RED_1 = np.array([0,   140, 140])
UPPER_RED_1 = np.array([12,  255, 255])
LOWER_RED_2 = np.array([168, 140, 140])
UPPER_RED_2 = np.array([180, 255, 255])

# Minimum mask area in pixels to count as a valid detection.
# If you are getting false negatives, lower this.
# If you are getting false positives (noise), raise this.
MIN_CONTOUR_AREA  = 300

FRAME_TIMEOUT_SEC = 0.5

# ==============================================================================


class GTGControllerNode(Node):

    def __init__(self):
        super().__init__('gtg_controller_node')

        self._load_calibration()

        self.active_camera       = 'realsense'
        self.last_realsense_time = 0

        self.bridge = CvBridge()

        self._latest_rs_frame:  np.ndarray | None = None
        self._last_rs_frame_ns: int = 0
        self._latest_arc_frame:  np.ndarray | None = None
        self._last_arc_frame_ns: int = 0

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

        # Debug mask publisher — subscribe with rqt_image_view to verify detection
        self.debug_mask_pub = self.create_publisher(
            Image, '/vision/gtg_debug_mask', SENSOR_QOS)

        # Frame subscriptions
        self.create_subscription(
            Image, '/vision/realsense_color', self._rs_frame_cb, SENSOR_QOS)
        self.create_subscription(
            Image, '/vision/arducam_raw', self._arc_frame_cb, SENSOR_QOS)

        self.create_timer(1.0 / 30.0, self._process_vision)
        self.get_logger().info("GTG controller initialised.")

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

    def _rs_frame_cb(self, msg: Image) -> None:
        self._latest_rs_frame  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self._last_rs_frame_ns = self.get_clock().now().nanoseconds

    def _arc_frame_cb(self, msg: Image) -> None:
        self._latest_arc_frame  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self._last_arc_frame_ns = self.get_clock().now().nanoseconds

    def _reset_pid(self) -> None:
        self.pid_steer.istate = self.pid_steer.dstate = self.pid_steer.error_prev = 0.0

    def _pixel_to_robot_frame(self, cX, cY, offset_x, offset_y,
                               pixels_per_meter, robot_x, robot_y):
        dx_px = cX - robot_x
        dy_px = robot_y - cY
        rel_x = (dx_px / pixels_per_meter) + offset_x
        rel_y = (dy_px / pixels_per_meter) + offset_y
        return rel_y, -rel_x   # x_robot, y_robot

    def get_red_centroid(self, frame: np.ndarray, publish_debug: bool = False):
        """
        Returns (cX, cY) of the largest red blob, or (None, None).

        When publish_debug=True, publishes the combined mask image to
        /vision/gtg_debug_mask so you can verify detection in rqt_image_view.
        """
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1),
            cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2),
        )
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        if publish_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                self.debug_mask_pub.publish(debug_msg)
            except Exception:
                pass

        M = cv2.moments(mask)
        if M['m00'] > MIN_CONTOUR_AREA:
            return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
        return None, None

    # -----------------------------------------------------------------------

    def _process_vision(self) -> None:
        now_ns          = self.get_clock().now().nanoseconds
        rs_target_found = False

        # 1. RealSense — highest priority
        rs_frame_age = (now_ns - self._last_rs_frame_ns) / 1e9
        if self._latest_rs_frame is not None and rs_frame_age < FRAME_TIMEOUT_SEC:

            # Always publish debug mask from RealSense so you can monitor it
            cX, cY = self.get_red_centroid(
                self._latest_rs_frame, publish_debug=True)

            if cX is not None:
                rs_target_found          = True
                self.last_realsense_time = now_ns

                if self.active_camera != 'realsense':
                    self.get_logger().info("RealSense regained detection.")
                    self._reset_pid()
                    self.active_camera = 'realsense'

                x_robot, y_robot = self._pixel_to_robot_frame(
                    cX, cY, REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y,
                    self.rs_pixels_per_meter, self.rs_robot_x, self.rs_robot_y)
                dist = math.sqrt(x_robot**2 + y_robot**2)

                self.get_logger().info(
                    f"[GTG RS] DETECTED centroid=({cX},{cY}) "
                    f"dist={dist:.2f}m",
                    throttle_duration_sec=0.5)

                self._run_control(x_robot, y_robot, dist, GOAL_THRESH_REALSENSE)

            else:
                # Log that we have a frame but no detection — visible every 2s
                self.get_logger().info(
                    f"[GTG RS] Frame OK, no red detected "
                    f"(mask area < {MIN_CONTOUR_AREA}px). "
                    "Check /vision/gtg_debug_mask in rqt_image_view.",
                    throttle_duration_sec=2.0)

        # 2. Arducam fallback
        realsense_age_sec = (now_ns - self.last_realsense_time) / 1e9
        arc_frame_age     = (now_ns - self._last_arc_frame_ns) / 1e9

        if (not rs_target_found
                and realsense_age_sec >= REALSENSE_TIMEOUT_SEC
                and self._latest_arc_frame is not None
                and arc_frame_age < FRAME_TIMEOUT_SEC):

            cX, cY = self.get_red_centroid(self._latest_arc_frame)

            if cX is not None:
                if self.active_camera != 'arducam':
                    self.get_logger().info(
                        f"RealSense lost {realsense_age_sec:.1f}s "
                        "— Arducam taking control.")
                    self._reset_pid()
                    self.active_camera = 'arducam'

                x_robot, y_robot = self._pixel_to_robot_frame(
                    cX, cY, ARDUCAM_OFFSET_X, ARDUCAM_OFFSET_Y,
                    self.arc_pixels_per_meter, self.arc_robot_x, self.arc_robot_y)
                dist = math.sqrt(x_robot**2 + y_robot**2)

                self.get_logger().info(
                    f"[GTG ARC] DETECTED centroid=({cX},{cY}) dist={dist:.2f}m",
                    throttle_duration_sec=0.5)

                self._run_control(x_robot, y_robot, dist, GOAL_THRESH_ARDUCAM)

    # -----------------------------------------------------------------------

    def _run_control(self, x_robot, y_robot, dist, goal_thresh) -> None:
        if dist <= goal_thresh:
            self.get_logger().info(
                f"Target reached (dist={dist:.2f}m). Triggering auger.")
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