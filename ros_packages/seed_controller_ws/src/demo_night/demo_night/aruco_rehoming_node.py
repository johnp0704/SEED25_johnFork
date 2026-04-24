"""
aruco_rehoming_node.py  —  ArUco rehoming via step-and-pause pivot scan.

Scan strategy
-------------
Instead of pivoting continuously (which causes motion blur and makes the
ArUco detector unreliable), the robot rotates in discrete steps:

  1. STEP:  Pivot CW at SCAN_PIVOT_SPEED for STEP_DURATION_SEC.
  2. PAUSE: Hold still for PAUSE_DURATION_SEC while the detector runs.
  3. If a new marker is visible during the pause, record it.
  4. Repeat until all 4 markers are found.

This gives the camera a stable, blur-free image during each detection
window regardless of how fast or slow the motors run.

States
------
IDLE      → publish nothing, wait for /rehome/reset
SCANNING  → step-and-pause pivot (sub-states: STEP / PAUSE)
ROTATING  → timed pivot to face the computed centre direction
CENTRE    → dead-reckoned straight drive to centre
ORIENT    → vision-servo pivot until marker 0 is centred
DONE      → hold [0, 0]
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
# Step-and-pause scan parameters
# ===========================================================================

# How long to pivot CW on each step.
# Keep short so the robot doesn't overshoot a marker.
STEP_DURATION_SEC  = 0.8    # seconds of active pivoting per step

# How long to hold still after each step for detection.
# Must be long enough for at least 2-3 camera frames to be processed.
# At 30 fps that's ~100 ms per frame, so 0.5 s gives ~15 frames.
PAUSE_DURATION_SEC = 0.6    # seconds of stillness per pause

# CW pivot speed during each step.
# Use a value you know overcomes motor stall from your optical follower tests.
SCAN_PIVOT_SPEED   = 35.0   # cmd units — matches optical follower magnitude

# ===========================================================================
# Other control parameters
# ===========================================================================

ROTATE_SPEED     = 35.0
CENTRE_SPEED     = 35.0
ORIENT_SPEED     = 35.0

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

    # Scan sub-states
    _SCAN_STEP  = "STEP"
    _SCAN_PAUSE = "PAUSE"

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

        # 30 Hz loop
        self.create_timer(1.0 / 30.0, self._control_loop)
        self.get_logger().info(
            "Rehoming Node ready. Waiting for /rehome/reset.")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _clear_scan_data(self) -> None:
        self._seen_markers:          dict[int, np.ndarray] = {}

        # Step-and-pause scan sub-state
        self._scan_sub_state:        str   = self._SCAN_STEP
        self._scan_phase_start_ns:   int   = 0   # when current sub-state began
        self._step_count:            int   = 0

        # Remaining phases
        self._centre_angle_rad:      float = 0.0
        self._centre_dist_m:         float = 0.0
        self._rotate_duration_s:     float = 0.0
        self._rotate_start_ns:       int   = 0
        self._centre_drive_start_ns: int   = 0

    def _publish_wheels(self, left: float, right: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _frame_is_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.last_frame_stamp_ns
        return age_ns < FRAME_TIMEOUT_SEC * 1e9

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame        = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    def _reset_cb(self, _msg: String) -> None:
        self.get_logger().info("[REHOME] Reset — beginning step-and-pause scan.")
        self._clear_scan_data()
        now_ns                     = self.get_clock().now().nanoseconds
        self._state                = self.STATE_SCANNING
        self._scan_sub_state       = self._SCAN_STEP
        self._scan_phase_start_ns  = now_ns
        self._publish_status("SCANNING")

    # -----------------------------------------------------------------------
    # ArUco detection
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

    # -----------------------------------------------------------------------
    # Centre offset computation
    # -----------------------------------------------------------------------

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
                f"  Marker {mid}: ({mx:.3f}, {mz:.3f})m "
                f"→ centre ({cx:.3f}, {cz:.3f})m")
        if not dx_list:
            return 0.0, 0.0
        return float(np.mean(dx_list)), float(np.mean(dz_list))

    # -----------------------------------------------------------------------
    # Main control loop — 30 Hz
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:

        if self._state == self.STATE_IDLE:
            return

        if self._state == self.STATE_DONE:
            self._publish_wheels(0.0, 0.0)
            return

        # ROTATING and CENTRE are dead-reckoning — don't block on frame freshness
        if self._state == self.STATE_ROTATING:
            self._do_rotating()
            return

        if self._state == self.STATE_CENTRE:
            self._do_centre()
            return

        # SCANNING and ORIENT need camera frames
        if not self._frame_is_fresh() or self.latest_frame is None:
            self.get_logger().warn(
                "No fresh Arducam frame — stopping for safety.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        if self._state == self.STATE_SCANNING:
            self._do_scanning()
        elif self._state == self.STATE_ORIENT:
            self._do_orient(self._detect_markers(self.latest_frame))

    # -----------------------------------------------------------------------
    # SCANNING — step-and-pause
    # -----------------------------------------------------------------------

    def _do_scanning(self) -> None:
        now_ns    = self.get_clock().now().nanoseconds
        elapsed_s = (now_ns - self._scan_phase_start_ns) / 1e9

        # ---- STEP sub-state: pivot CW ----
        if self._scan_sub_state == self._SCAN_STEP:
            if elapsed_s < STEP_DURATION_SEC:
                # Actively pivoting — publish CW command every tick
                # CW: left forward (+), right backward (-)
                self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)
                self.get_logger().info(
                    f"[SCAN STEP {self._step_count}] "
                    f"pivoting {elapsed_s:.2f}/{STEP_DURATION_SEC:.2f}s",
                    throttle_duration_sec=0.5)
            else:
                # Step complete — transition to PAUSE
                self._publish_wheels(0.0, 0.0)
                self._scan_sub_state      = self._SCAN_PAUSE
                self._scan_phase_start_ns = now_ns
                self.get_logger().info(
                    f"[SCAN STEP {self._step_count}] done — pausing for detection.")

        # ---- PAUSE sub-state: hold still and detect ----
        elif self._scan_sub_state == self._SCAN_PAUSE:
            # Always hold motors at zero during pause
            self._publish_wheels(0.0, 0.0)

            # Run detection on every tick during the pause window
            visible = self._detect_markers(self.latest_frame)
            for mid, tvec in visible.items():
                if mid not in self._seen_markers:
                    self._seen_markers[mid] = tvec
                    self.get_logger().info(
                        f"[SCAN PAUSE {self._step_count}] "
                        f"Marker {mid} recorded: "
                        f"x={tvec[0]:.3f}m  z={tvec[2]:.3f}m  "
                        f"({len(self._seen_markers)}/4 total)")
                    self._publish_status(f"FOUND:{mid}")

            # Check completion
            if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
                self.get_logger().info(
                    "[SCANNING] All 4 markers found — computing centre.")
                self._begin_rotating()
                return

            # Pause window expired — start next step
            if elapsed_s >= PAUSE_DURATION_SEC:
                self._step_count         += 1
                self._scan_sub_state      = self._SCAN_STEP
                self._scan_phase_start_ns = now_ns
                self.get_logger().info(
                    f"[SCAN] Starting step {self._step_count}. "
                    f"Markers found so far: {list(self._seen_markers.keys())}")

    # -----------------------------------------------------------------------
    # ROTATING
    # -----------------------------------------------------------------------

    def _begin_rotating(self) -> None:
        self._publish_wheels(0.0, 0.0)

        dx, dz = self._compute_centre_offset()
        dist   = math.sqrt(dx**2 + dz**2)

        if dist < CENTRE_ARRIVAL_M:
            self.get_logger().info("[ROTATING] Already at centre → ORIENT")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        angle = math.atan2(dx, dz)   # + = centre to the right → CW
        self.get_logger().info(
            f"[ROTATING] dx={dx:.3f}m  dz={dz:.3f}m  "
            f"dist={dist:.3f}m  angle={math.degrees(angle):.1f}°")

        self._centre_angle_rad  = angle
        self._centre_dist_m     = dist
        omega = (2.0 * ROTATE_SPEED * self._mps_per_cmd) / WHEEL_BASE_M
        self._rotate_duration_s = abs(angle) / omega if omega > 0 else 0.0

        self.get_logger().info(
            f"[ROTATING] {math.degrees(angle):.1f}° for "
            f"{self._rotate_duration_s:.2f}s, then drive {dist:.3f}m.")

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
        # CW: left fwd, right back.  CCW: left back, right fwd.
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
                "[ORIENT] NORTH not visible — CCW search.",
                throttle_duration_sec=1.0)
            # CCW: left back, right fwd
            self._publish_wheels(-ORIENT_SPEED, ORIENT_SPEED)
            return

        tvec        = visible[0]
        x_offset    = float(tvec[0])
        z_dist      = max(float(tvec[2]), 0.1)
        angle_error = math.atan2(x_offset, z_dist)

        self.get_logger().info(
            f"[ORIENT] x={x_offset:.4f}m  "
            f"angle={math.degrees(angle_error):.1f}°",
            throttle_duration_sec=0.5)

        if abs(angle_error) <= ORIENT_TOL_RAD:
            self.get_logger().info("[ORIENT] Aligned — DONE.")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        # CW: left fwd, right back.  CCW: left back, right fwd.
        if angle_error > 0:
            self._publish_wheels( ORIENT_SPEED, -ORIENT_SPEED)
        else:
            self._publish_wheels(-ORIENT_SPEED,  ORIENT_SPEED)


# ===========================================================================

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