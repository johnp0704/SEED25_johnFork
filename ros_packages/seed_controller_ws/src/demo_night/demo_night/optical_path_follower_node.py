"""
optical_path_following_node.py  —  Blue tape follower using Arducam feed.
Update LOWER_BLUE / UPPER_BLUE with values from arducam_color_sampler.py.

Key fixes in this version
--------------------------
1. Startup grace period.
   The node waits STARTUP_GRACE_FRAMES frames before allowing recovery spin.
   This prevents a single stale/missed frame at startup from immediately
   flinging the robot into a spin before it has had a chance to settle and
   see the tape that is right in front of it.

2. Recovery spin direction changed to CCW.
   CW spin was driving the robot toward the background tables.
   CCW (left backward, right forward) searches in the other direction.
   If CCW is also wrong for your arena layout, set RECOVERY_SPIN_DIR = 1.0
   to go back to CW.

3. PID and counters are explicitly zeroed at __init__ time so there is no
   ambiguity about initial state regardless of when the first frame arrives.

4. BOTTOM_HALF_FRACTION = 0.70 is kept — the math is correct:
       mask[:cutoff_row, :] = 0   keeps rows [cutoff_row .. h-1]
   i.e. the bottom 30 % of the frame.  Tune this if the tape is being
   clipped (raise fraction) or background sneaks in (lower fraction).
"""
from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from ml_red_controller.PID import PID

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

# ===========================================================================
# Colour thresholds — UPDATE from arducam_color_sampler.py
# ===========================================================================

LOWER_BLUE = np.array([69,  16, 159])
UPPER_BLUE = np.array([110, 103, 223])

# ===========================================================================
# Detection parameters
# ===========================================================================

MIN_TAPE_AREA = 800

# Rows above this fraction of frame height are ignored.
# 0.70 → keeps only the bottom 30 % of the frame.
# Raise toward 1.0 to look higher up; lower toward 0.5 to be more aggressive.
BOTTOM_HALF_FRACTION = 0.70

# ===========================================================================
# Control parameters
# ===========================================================================

BASE_SPEED         = 35.0
MAX_ACTUATOR_INPUT = 50.0

PID_KP  = 0.25
PID_KI  = 0.01
PID_KD  = 0.06
PID_N   = 10.0
PID_KAW = 0.5

# ===========================================================================
# Recovery spin
# ===========================================================================

RECOVERY_SPIN_SPEED = 30.0

# Direction of recovery spin when tape is lost.
#  -1.0 = CCW  (left backward, right forward)  ← default, away from tables
#  +1.0 = CW   (left forward, right backward)
RECOVERY_SPIN_DIR = -1.0

# Number of consecutive "no tape" frames before recovery spin begins.
# This is the critical startup guard: the robot won't spin on the very first
# tick if a single frame is missed or arrives stale.
# At 30 Hz, 10 frames = ~0.33 s grace period before spin starts.
STARTUP_GRACE_FRAMES = 10

# ===========================================================================
# Timing
# ===========================================================================

FRAME_TIMEOUT_SEC    = 1.0
INTERRUPTION_GAP_SEC = 0.5   # PID reset if optical was paused longer than this


class OpticalPathNode(Node):

    def __init__(self):
        super().__init__('optical_path_follower_node')

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/vision/optical_cmd', 10)

        self.bridge         = CvBridge()
        self.latest_frame:  np.ndarray | None = None
        self.last_frame_ns: int = 0

        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1.0 / 30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW,
        )

        # Explicit zero of all PID state at init — no ambiguity.
        self._reset_pid()

        self._lost_frames  = 0
        self._last_tick_ns = 0

        self.create_timer(1.0 / 30.0, self._control_loop)
        self.get_logger().info(
            f"Optical Path Tracking initialised.  "
            f"Recovery spin: {'CCW' if RECOVERY_SPIN_DIR < 0 else 'CW'} "
            f"after {STARTUP_GRACE_FRAMES} missed frames.")

    # -----------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_ns = self.get_clock().now().nanoseconds

    def _reset_pid(self) -> None:
        self.pid_steer.istate             = 0.0
        self.pid_steer.dstate             = 0.0
        self.pid_steer.error_prev         = 0.0
        self.pid_steer.actuatorError_prev = 0.0

    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:
        now_ns  = self.get_clock().now().nanoseconds
        age_sec = (now_ns - self.last_frame_ns) / 1e9

        if self.latest_frame is None or age_sec > FRAME_TIMEOUT_SEC:
            self.get_logger().warn(
                "No fresh Arducam frame — halting.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            self._last_tick_ns = 0   # force PID reset on next real tick
            return

        # -------------------------------------------------------------------
        # PID reset if this node was interrupted (GTG, IDLE, etc.)
        # -------------------------------------------------------------------
        if self._last_tick_ns != 0:
            gap_sec = (now_ns - self._last_tick_ns) / 1e9
            if gap_sec > INTERRUPTION_GAP_SEC:
                self._reset_pid()
                self._lost_frames = 0
                self.get_logger().info(
                    f"[OPTICAL] PID reset after {gap_sec:.2f}s interruption gap.")

        self._last_tick_ns = now_ns

        frame = self.latest_frame
        h, w  = frame.shape[:2]

        # -------------------------------------------------------------------
        # Mask — bottom portion only.
        # cutoff_row is the first row we KEEP; everything above is zeroed.
        # Example: h=480, BOTTOM_HALF_FRACTION=0.70 → cutoff_row=336
        #   mask[0:336, :] = 0   (top 70 % ignored)
        #   mask[336:480, :] unchanged  (bottom 30 % kept)
        # -------------------------------------------------------------------
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cutoff_row = int(h * BOTTOM_HALF_FRACTION)
        mask[:cutoff_row, :] = 0

        # -------------------------------------------------------------------
        # Contour detection — lowest centroid wins.
        # -------------------------------------------------------------------
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid = [c for c in contours if cv2.contourArea(c) >= MIN_TAPE_AREA]

        if valid:
            def contour_cy(c):
                M = cv2.moments(c)
                return int(M['m01'] / M['m00']) if M['m00'] > 0 else 0

            best = max(valid, key=contour_cy)
            M    = cv2.moments(best)

            if self._lost_frames > 0:
                self.get_logger().info(
                    f"[OPTICAL] Tape re-acquired after {self._lost_frames} frames.")
            self._lost_frames = 0

            cx      = int(M['m10'] / M['m00'])
            cy      = int(M['m01'] / M['m00'])
            error_x = (w / 2.0) - cx

            self.get_logger().info(
                f"[OPTICAL] centroid=({cx},{cy})  error_x={error_x:.1f}  "
                f"blobs={len(valid)}",
                throttle_duration_sec=0.5)

            correction = self.pid_steer.update(setpoint=error_x, output=0.0)

            wl = BASE_SPEED - correction
            wr = BASE_SPEED + correction
            mag = max(abs(wl), abs(wr))
            if mag > MAX_ACTUATOR_INPUT:
                wl *= MAX_ACTUATOR_INPUT / mag
                wr *= MAX_ACTUATOR_INPUT / mag

            self._publish_wheels(wl, wr)

        else:
            self._lost_frames += 1

            # ---------------------------------------------------------------
            # STARTUP GRACE PERIOD: don't spin until we've missed at least
            # STARTUP_GRACE_FRAMES consecutive frames.  This gives the camera
            # pipeline time to deliver the first real frame and prevents a
            # single startup miss from launching a recovery spin immediately.
            # ---------------------------------------------------------------
            if self._lost_frames <= STARTUP_GRACE_FRAMES:
                self.get_logger().info(
                    f"[OPTICAL] No tape yet — grace period "
                    f"({self._lost_frames}/{STARTUP_GRACE_FRAMES}).",
                    throttle_duration_sec=0.5)
                self._publish_wheels(0.0, 0.0)
            else:
                self.get_logger().warn(
                    f"[OPTICAL] Tape lost ({self._lost_frames} frames) "
                    f"— spinning {'CCW' if RECOVERY_SPIN_DIR < 0 else 'CW'}.",
                    throttle_duration_sec=2.0)
                wl = -RECOVERY_SPIN_SPEED * RECOVERY_SPIN_DIR
                wr =  RECOVERY_SPIN_SPEED * RECOVERY_SPIN_DIR
                self._publish_wheels(wl, wr)

    # -----------------------------------------------------------------------

    def _publish_wheels(self, left: float, right: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)


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