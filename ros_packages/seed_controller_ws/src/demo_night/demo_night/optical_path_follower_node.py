"""
optical_path_following_node.py

Follows a loop of blue painter's tape on the ground using the RealSense
colour feed published by realsense_node.py.

Key fixes vs. the original
---------------------------
* No longer opens the RealSense pipeline directly — subscribes to
  /vision/realsense_color instead (resolves the pipeline conflict).
* PID sign convention is consistent and documented.
* Motor mixing signs verified against the Sabertooth convention.
* Line-lost recovery: slow pivot search instead of a hard stop.

Topics
------
Subscribes : /vision/realsense_color  (sensor_msgs/Image)
Publishes  : /vision/optical_cmd      (std_msgs/Float32MultiArray)
             /vision/arducam_display  (sensor_msgs/Image)  ← optional debug feed
"""
from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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

# PID import — adjust package path as needed
from ml_red_controller.PID import PID


# ===========================================================================
# Tuning constants
# ===========================================================================

# HSV range for the blue painter's tape.
# Use realsense_color_sampler.py to refine these for your specific tape/light.
LOWER_BLUE = np.array([97, 142, 188])
UPPER_BLUE = np.array([100, 255, 226])

# Minimum contour area (pixels²) to be considered a valid tape detection.
MIN_TAPE_AREA = 800

# Fraction of the image height used as the look-ahead row band.
# 0.5 → bottom half; 0.7 → bottom 30% (tighter look-ahead).
LOOK_AHEAD_FRACTION = 0.55   # use rows below this fraction of image height

# Drive speeds (Sabertooth command units, 0–100)
BASE_SPEED         = 35.0
MAX_ACTUATOR_INPUT = 50.0

# PID steering — tuned for 30 Hz, output in command units
PID_KP  = 0.25
PID_KI  = 0.01
PID_KD  = 0.06
PID_N   = 10.0
PID_KAW = 0.5

# Recovery pivot: slow spin speed when tape is lost
RECOVERY_PIVOT_SPEED = 20.0

# After this many consecutive lost frames, switch pivot direction
RECOVERY_FLIP_FRAMES = 45   # ~1.5 s at 30 Hz

# Frame age limit — if no new frame arrives within this window, stop.
FRAME_TIMEOUT_SEC = 0.5

# ===========================================================================


class OpticalPathNode(Node):

    def __init__(self):
        super().__init__('optical_path_node')

        # Publishers
        self.cmd_pub     = self.create_publisher(
            Float32MultiArray, '/vision/optical_cmd', 10)

        # Subscribe to frames from the central RealSense node
        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_ns: int = 0
        self.create_subscription(
            Image, '/vision/realsense_color', self._frame_cb, SENSOR_QOS)

        # Steering PID
        # Convention:
        #   error = image_centre_x − tape_centroid_x
        #   positive error → tape is LEFT of centre → need to steer LEFT
        #     → left wheel slows, right wheel speeds up (for a forward-driving robot)
        #   PID output (correction) is positive when error is positive
        #   wl = base − correction   (slow down left to turn left)
        #   wr = base + correction   (speed up right to turn left)
        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1.0 / 30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW,
        )

        # Recovery state
        self._lost_frames      = 0
        self._recovery_dir     = 1   # +1 = CW,  -1 = CCW

        # Control loop at 30 Hz
        self.create_timer(1.0 / 30.0, self._control_loop)
        self.get_logger().info("Optical Path Tracking initialised.")

    # -----------------------------------------------------------------------
    # Frame subscription
    # -----------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame_ns = self.get_clock().now().nanoseconds

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:
        now_ns   = self.get_clock().now().nanoseconds
        age_sec  = (now_ns - self.last_frame_ns) / 1e9

        if self.latest_frame is None or age_sec > FRAME_TIMEOUT_SEC:
            self.get_logger().warn(
                "No fresh RealSense frame — halting.", throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        frame = self.latest_frame
        h, w  = frame.shape[:2]

        # --- Colour mask ------------------------------------------------
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

        # Morphological clean-up
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Restrict to the look-ahead zone (lower portion of image)
        look_ahead_row = int(h * LOOK_AHEAD_FRACTION)
        mask[:look_ahead_row, :] = 0

        # --- Centroid ---------------------------------------------------
        M = cv2.moments(mask)
        area = M['m00']

        if area >= MIN_TAPE_AREA:
            # Tape found — reset recovery state
            self._lost_frames  = 0
            self._recovery_dir = 1
            self.get_logger().debug("Tape detected.")

            cx = int(M['m10'] / area)

            # Signed error: positive → tape is LEFT of centre
            error_x = (w / 2.0) - cx

            # PID: setpoint = 0 (tape centred), output = current error
            # error inside PID = setpoint − output = 0 − error_x = −error_x
            # To keep the sign meaningful, pass error_x as the output directly:
            #   PID internally computes: e = 0 − error_x → correction follows
            # This is correct: if tape is left (error_x > 0), correction < 0
            # which would slow the right wheel — WRONG direction.
            #
            # Instead, pass error_x as setpoint and 0 as output so that:
            #   e = error_x − 0 = error_x (positive → tape left → turn left)
            correction = self.pid_steer.update(setpoint=error_x, output=0.0)

            # Motor mixing (Sabertooth convention: positive = forward)
            # Turn left  (correction > 0): slow left, speed right → No.
            # Turning left for a differential drive means RIGHT wheel faster.
            # wl = base − correction  (if correction > 0: left slows → turns left ✓)
            # wr = base + correction
            wl = BASE_SPEED - correction
            wr = BASE_SPEED + correction

            # Clamp
            max_mag = max(abs(wl), abs(wr))
            if max_mag > MAX_ACTUATOR_INPUT:
                scale = MAX_ACTUATOR_INPUT / max_mag
                wl   *= scale
                wr   *= scale

            self._publish_wheels(wl, wr)

        else:
            # Tape lost — execute recovery pivot
            self._lost_frames += 1
            self.get_logger().warn(
                f"Tape lost for {self._lost_frames} frames — "
                f"recovery pivot {'CW' if self._recovery_dir > 0 else 'CCW'}.",
                throttle_duration_sec=1.0,
            )

            # Flip pivot direction after RECOVERY_FLIP_FRAMES to sweep wider
            if self._lost_frames % RECOVERY_FLIP_FRAMES == 0:
                self._recovery_dir *= -1
                self.get_logger().info(
                    f"Recovery direction flipped → "
                    f"{'CW' if self._recovery_dir > 0 else 'CCW'}"
                )

            # CW pivot: left forward, right backward
            pivot = RECOVERY_PIVOT_SPEED * self._recovery_dir
            self._publish_wheels(pivot, -pivot)

    # -----------------------------------------------------------------------
    # Publisher helper
    # -----------------------------------------------------------------------

    def _publish_wheels(self, left: float, right: float) -> None:
        msg      = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OpticalPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()