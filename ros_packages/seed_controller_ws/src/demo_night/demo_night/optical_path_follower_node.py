"""
optical_path_following_node.py  —  Blue tape follower using Arducam feed.
Update LOWER_BLUE / UPPER_BLUE with values from arducam_color_sampler.py.
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
# Tuning constants — UPDATE LOWER_BLUE / UPPER_BLUE from arducam_color_sampler
# ===========================================================================

LOWER_BLUE = np.array([102, 108, 200])   # ← replace with Arducam-sampled values
UPPER_BLUE = np.array([107, 136, 225])  # ← replace with Arducam-sampled values

MIN_TAPE_AREA       = 800
LOOK_AHEAD_FRACTION = 0.55

BASE_SPEED         = 35.0
MAX_ACTUATOR_INPUT = 50.0

PID_KP  = 0.25
PID_KI  = 0.01
PID_KD  = 0.06
PID_N   = 10.0
PID_KAW = 0.5

# Recovery: continuous full rotation at this speed until tape is re-acquired.
# One full rotation takes approximately (360 / angular_velocity) seconds.
# At speed 30, a typical differential drive robot completes a rotation in ~6s.
RECOVERY_SPIN_SPEED = 30.0

FRAME_TIMEOUT_SEC = 1.0


class OpticalPathNode(Node):

    def __init__(self):
        super().__init__('optical_path_follower_node')

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, '/vision/optical_cmd', 10)

        self.bridge        = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_ns: int = 0

        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1.0 / 30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW,
        )

        self._lost_frames = 0

        self.create_timer(1.0 / 30.0, self._control_loop)
        self.get_logger().info("Optical Path Tracking initialised (Arducam).")

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame_ns = self.get_clock().now().nanoseconds

    def _control_loop(self) -> None:
        now_ns  = self.get_clock().now().nanoseconds
        age_sec = (now_ns - self.last_frame_ns) / 1e9

        if self.latest_frame is None or age_sec > FRAME_TIMEOUT_SEC:
            self.get_logger().warn(
                "No fresh Arducam frame — halting.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        frame = self.latest_frame
        h, w  = frame.shape[:2]

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        look_ahead_row = int(h * LOOK_AHEAD_FRACTION)
        mask[:look_ahead_row, :] = 0

        M    = cv2.moments(mask)
        area = M['m00']

        if area >= MIN_TAPE_AREA:
            # Tape found — reset recovery counter and drive normally
            if self._lost_frames > 0:
                self.get_logger().info(
                    f"[OPTICAL] Tape re-acquired after {self._lost_frames} frames.")
            self._lost_frames = 0

            cx         = int(M['m10'] / area)
            error_x    = (w / 2.0) - cx
            correction = self.pid_steer.update(setpoint=error_x, output=0.0)

            wl = BASE_SPEED - correction
            wr = BASE_SPEED + correction
            mag = max(abs(wl), abs(wr))
            if mag > MAX_ACTUATOR_INPUT:
                wl *= MAX_ACTUATOR_INPUT / mag
                wr *= MAX_ACTUATOR_INPUT / mag

            self._publish_wheels(wl, wr)

        else:
            # Tape lost — spin CW continuously until re-acquired.
            # A single consistent direction ensures a full sweep rather than
            # oscillating back and forth over the same small arc.
            self._lost_frames += 1
            self.get_logger().warn(
                f"[OPTICAL] Tape lost for {self._lost_frames} frames — "
                "spinning CW for full recovery rotation.",
                throttle_duration_sec=2.0)

            # CW: left wheel forward, right wheel backward
            self._publish_wheels(RECOVERY_SPIN_SPEED, -RECOVERY_SPIN_SPEED)

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