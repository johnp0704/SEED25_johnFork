"""
aruco_rehoming_node.py  —  ArUco rehoming via single pivot scan.

Motor control is now structured to exactly mirror the working
optical_path_following_node.py:
  - Publishes on every single timer tick (no conditional skipping)
  - Uses the same topic, same message type, same Hz target (30 Hz)
  - Dwell guard only suppresses marker *recording*, not command publishing
  - Added diagnostic logging of the raw left/right values so you can
    confirm in the terminal that non-zero commands are actually being sent

States
------
IDLE      → publish nothing, wait for /rehome/reset
SCANNING  → publish CW pivot every tick; record markers after dwell window
ROTATING  → publish timed pivot to face centre
CENTRE    → publish forward drive for dead-reckoned distance
ORIENT    → publish pivot until marker 0 is centred
DONE      → publish [0, 0]
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

import cv2
import cv2.aruco as aruco
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

# ===========================================================================
# Physical constants
# ===========================================================================

WALL_DISTANCE_TO_CENTER: dict[int, float] = {
    0: 2.1336,
    1: 1.3208,
    2: 1.81769,
    3: 0.9017,
}
MARKER_LENGTH_M = 0.0854

# ===========================================================================
# Speeds — deliberately matched to optical follower magnitudes
# ===========================================================================

# The optical follower uses BASE_SPEED=35 and it works.
# Use the same magnitude for rehoming pivots so we know the motor path works.
SCAN_PIVOT_SPEED = 35.0   # raised from 15 to match optical follower magnitude
ROTATE_SPEED     = 35.0
CENTRE_SPEED     = 35.0
ORIENT_SPEED     = 35.0

# Dwell: how long to spin before accepting any marker detection.
# Keeps the robot moving before recording tvecs.
PIVOT_DWELL_SEC  = 1.0

ORIENT_TOL_RAD   = math.radians(5.0)
CENTRE_ARRIVAL_M = 0.08

FRAME_TIMEOUT_SEC = 1.0
WHEEL_BASE_M      = 0.40

DR_CAL_PATH = (
    "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
    "demo_night/demo_night/dead_reckoning_cal.npz"
)

ARDUCAM_CAMERA_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
ARDUCAM_DIST_COEFFS = np.zeros(5, dtype=np.float64)

# ===========================================================================


class RehomeNode(Node):

    STATE_IDLE     = "IDLE"
    STATE_SCANNING = "SCANNING"
    STATE_ROTATING = "ROTATING"
    STATE_CENTRE   = "CENTRE"
    STATE_ORIENT   = "ORIENT"
    STATE_DONE     = "DONE"

    def __init__(self):
        super().__init__('aruco_rehoming_node')

        self.cmd_pub    = self.create_publisher(
            Float32MultiArray, '/vision/rehome_cmd', 10)
        self.status_pub = self.create_publisher(String, '/rehome/status', 10)

        self.bridge                          = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_stamp_ns: int        = 0

        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)
        self.create_subscription(
            String, '/rehome/reset', self._reset_cb, 10)

        self.camera_matrix = ARDUCAM_CAMERA_MATRIX.copy()
        self.dist_coeffs   = ARDUCAM_DIST_COEFFS.copy()

        if os.path.exists(DR_CAL_PATH):
            _dr = np.load(DR_CAL_PATH)
            self._mps_per_cmd = float(_dr['ratio'])
            self.get_logger().info(
                f"Dead-reckoning cal loaded: {self._mps_per_cmd:.5f} m/s/cmd")
        else:
            self._mps_per_cmd = 0.005
            self.get_logger().warn("DR cal not found — using 0.005 m/s/cmd")

        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        self._state = self.STATE_IDLE
        self._clear_scan_data()

        # 30 Hz — same as optical follower
        self.create_timer(1.0 / 30.0, self._control_loop)
        self.get_logger().info("Rehoming Node ready. Waiting for /rehome/reset.")

    # -----------------------------------------------------------------------

    def _clear_scan_data(self) -> None:
        self._seen_markers:          dict[int, np.ndarray] = {}
        self._scan_start_ns:         int   = 0
        self._centre_angle_rad:      float = 0.0
        self._centre_dist_m:         float = 0.0
        self._rotate_duration_s:     float = 0.0
        self._rotate_start_ns:       int   = 0
        self._centre_drive_start_ns: int   = 0

    def _publish_wheels(self, left: float, right: float) -> None:
        """
        Publish wheel command.  Logs at DEBUG level every tick so you can
        confirm non-zero values are actually being sent during the pivot.
        Watch for these lines in: ros2 topic echo /vision/rehome_cmd
        """
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)
        self.get_logger().debug(
            f"[REHOME CMD] L={left:.1f}  R={right:.1f}",
            throttle_duration_sec=0.5)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _frame_is_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.last_frame_stamp_ns
        return age_ns < FRAME_TIMEOUT_SEC * 1e9

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame        = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    def _reset_cb(self, _msg: String) -> None:
        self.get_logger().info("[REHOME] Reset — starting pivot scan.")
        self._clear_scan_data()
        self._state         = self.STATE_SCANNING
        self._scan_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("SCANNING")

    # -----------------------------------------------------------------------

    def _detect_markers(self, frame: np.ndarray) -> dict[int, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        results: dict[int, np.ndarray] = {}
        if ids is None:
            return results

        half    = MARKER_LENGTH_M / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        for i, mid_raw in enumerate(ids.flatten()):
            mid = int(mid_raw)
            if mid not in WALL_DISTANCE_TO_CENTER:
                continue
            img_pts = corners[i].reshape(4, 2).astype(np.float64)
            ok, _, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                results[mid] = tvec.flatten()
        return results

    def _compute_centre_offset(self) -> tuple[float, float]:
        dx_list: list[float] = []
        dz_list: list[float] = []
        for mid, tvec in self._seen_markers.items():
            mx   = float(tvec[0])
            mz   = float(tvec[2])
            dist = math.sqrt(mx**2 + mz**2)
            if dist < 0.01:
                continue
            ux = mx / dist
            uz = mz / dist
            d  = WALL_DISTANCE_TO_CENTER[mid]
            cx = mx - d * ux
            cz = mz - d * uz
            dx_list.append(cx)
            dz_list.append(cz)
            self.get_logger().info(
                f"  Marker {mid}: ({mx:.3f}, {mz:.3f})m → centre ({cx:.3f}, {cz:.3f})m")
        if not dx_list:
            return 0.0, 0.0
        return float(np.mean(dx_list)), float(np.mean(dz_list))

    # -----------------------------------------------------------------------
    # Main loop — 30 Hz, mirrors optical follower structure exactly
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:

        # IDLE: publish absolutely nothing
        if self._state == self.STATE_IDLE:
            return

        # DONE: hold zero
        if self._state == self.STATE_DONE:
            self._publish_wheels(0.0, 0.0)
            return

        # ROTATING and CENTRE do not need a camera frame — run them first
        # so a stale frame never blocks dead-reckoning phases.
        if self._state == self.STATE_ROTATING:
            self._do_rotating()
            return

        if self._state == self.STATE_CENTRE:
            self._do_centre()
            return

        # SCANNING and ORIENT need a fresh frame
        if not self._frame_is_fresh() or self.latest_frame is None:
            self.get_logger().warn(
                "No fresh Arducam frame — stopping for safety.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        visible = self._detect_markers(self.latest_frame)

        if self._state == self.STATE_SCANNING:
            self._do_scanning(visible)
        elif self._state == self.STATE_ORIENT:
            self._do_orient(visible)

    # -----------------------------------------------------------------------
    # SCANNING
    # -----------------------------------------------------------------------

    def _do_scanning(self, visible: dict[int, np.ndarray]) -> None:
        now_ns    = self.get_clock().now().nanoseconds
        dwell_sec = (now_ns - self._scan_start_ns) / 1e9

        # *** Publish the pivot command unconditionally on every tick ***
        # This mirrors optical_path_following_node exactly: it always
        # publishes before doing any logic that could change state.
        # CW pivot: left forward (+), right backward (-)
        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

        # Only start recording markers after the dwell window
        if dwell_sec < PIVOT_DWELL_SEC:
            self.get_logger().info(
                f"[SCANNING] Spinning up: {dwell_sec:.2f}/{PIVOT_DWELL_SEC:.2f}s",
                throttle_duration_sec=0.5)
            return

        for mid, tvec in visible.items():
            if mid not in self._seen_markers:
                self._seen_markers[mid] = tvec
                self.get_logger().info(
                    f"[SCANNING] Marker {mid}: x={tvec[0]:.3f} z={tvec[2]:.3f}m "
                    f"({len(self._seen_markers)}/4)")
                self._publish_status(f"FOUND:{mid}")

        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info("[SCANNING] All 4 found — computing centre.")
            self._begin_rotating()

    # -----------------------------------------------------------------------
    # ROTATING
    # -----------------------------------------------------------------------

    def _begin_rotating(self) -> None:
        # Stop first — this is the transition point
        self._publish_wheels(0.0, 0.0)

        dx, dz = self._compute_centre_offset()
        dist   = math.sqrt(dx**2 + dz**2)

        if dist < CENTRE_ARRIVAL_M:
            self.get_logger().info("[ROTATING] Already at centre → ORIENT")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        angle = math.atan2(dx, dz)   # + = centre is to the right → CW
        self.get_logger().info(
            f"[ROTATING] dx={dx:.3f} dz={dz:.3f} dist={dist:.3f}m "
            f"angle={math.degrees(angle):.1f}°")

        self._centre_angle_rad = angle
        self._centre_dist_m    = dist

        omega = (2.0 * ROTATE_SPEED * self._mps_per_cmd) / WHEEL_BASE_M
        self._rotate_duration_s = abs(angle) / omega if omega > 0 else 0.0
        self.get_logger().info(
            f"[ROTATING] {math.degrees(angle):.1f}° → {self._rotate_duration_s:.2f}s "
            f"then drive {dist:.3f}m")

        self._state           = self.STATE_ROTATING
        self._rotate_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("ROTATING")

    def _do_rotating(self) -> None:
        elapsed_s = (self.get_clock().now().nanoseconds
                     - self._rotate_start_ns) / 1e9
        if elapsed_s >= self._rotate_duration_s:
            self.get_logger().info("[ROTATING] Done → CENTRE")
            self._publish_wheels(0.0, 0.0)
            self._centre_drive_start_ns = self.get_clock().now().nanoseconds
            self._state = self.STATE_CENTRE
            self._publish_status("CENTERING")
            return
        # CW: left fwd, right back; CCW: left back, right fwd
        if self._centre_angle_rad >= 0:
            self._publish_wheels( ROTATE_SPEED, -ROTATE_SPEED)
        else:
            self._publish_wheels(-ROTATE_SPEED,  ROTATE_SPEED)

    # -----------------------------------------------------------------------
    # CENTRE
    # -----------------------------------------------------------------------

    def _do_centre(self) -> None:
        elapsed_s       = (self.get_clock().now().nanoseconds
                           - self._centre_drive_start_ns) / 1e9
        distance_driven = elapsed_s * CENTRE_SPEED * self._mps_per_cmd
        self.get_logger().info(
            f"[CENTRE] {distance_driven:.3f}/{self._centre_dist_m:.3f}m",
            throttle_duration_sec=0.5)
        if distance_driven >= self._centre_dist_m:
            self.get_logger().info("[CENTRE] Arrived → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
        else:
            self._publish_wheels(CENTRE_SPEED, CENTRE_SPEED)

    # -----------------------------------------------------------------------
    # ORIENT
    # -----------------------------------------------------------------------

    def _do_orient(self, visible: dict[int, np.ndarray]) -> None:
        if 0 not in visible:
            self.get_logger().info(
                "[ORIENT] NORTH not visible — CCW search",
                throttle_duration_sec=1.0)
            self._publish_wheels(-ORIENT_SPEED, ORIENT_SPEED)
            return

        tvec        = visible[0]
        x_offset    = float(tvec[0])
        z_dist      = max(float(tvec[2]), 0.1)
        angle_error = math.atan2(x_offset, z_dist)

        self.get_logger().info(
            f"[ORIENT] x={x_offset:.4f}m angle={math.degrees(angle_error):.1f}°",
            throttle_duration_sec=0.5)

        if abs(angle_error) <= ORIENT_TOL_RAD:
            self.get_logger().info("[ORIENT] Aligned — DONE")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        if angle_error > 0:
            self._publish_wheels( ORIENT_SPEED, -ORIENT_SPEED)  # CW
        else:
            self._publish_wheels(-ORIENT_SPEED,  ORIENT_SPEED)  # CCW


def main(args=None):
    rclpy.init(args=args)
    node = RehomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()