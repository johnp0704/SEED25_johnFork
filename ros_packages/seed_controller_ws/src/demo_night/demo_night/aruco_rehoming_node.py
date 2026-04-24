"""
aruco_rehoming_node.py  —  ArUco rehoming state machine using Arducam feed.
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

WALL_DISTANCE_TO_CENTER = {
    0: 2.1336,    # NORTH
    1: 1.3208,    # EAST
    2: 1.81769,   # SOUTH
    3: 0.9017,    # WEST
}

WALL_BEARING_FROM_CENTER = {
    0:  0.0,
    1: -math.pi / 2,
    2:  math.pi,
    3:  math.pi / 2,
}

MARKER_LENGTH_M = 0.0854

# ===========================================================================
# Control parameters
# ===========================================================================

STANDOFF_M            = 0.80
STANDOFF_TOL_M        = 0.05
CENTRE_ARRIVAL_M      = 0.10
ORIENT_TOL_RAD        = math.radians(5.0)

SCAN_PIVOT_SPEED      = 25.0
APPROACH_SPEED        = 35.0
CENTRE_SPEED          = 38.0
ORIENT_SPEED          = 28.0
KP_LATERAL            = 40.0

MIN_MARKERS_FOR_CENTRE = 4
FRAME_TIMEOUT_SEC      = 1.0   # Arducam V4L2 can be slower than RealSense

# Dead-reckoning calibration path
DR_CAL_PATH = (
    "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
    "demo_night/demo_night/dead_reckoning_cal.npz"
)

# Arducam intrinsics — replace with values from a checkerboard calibration
# if you have them; these defaults give reasonable ArUco pose for a typical
# 640×480 webcam-class sensor.
ARDUCAM_CAMERA_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
ARDUCAM_DIST_COEFFS = np.zeros(5, dtype=np.float64)

# ===========================================================================


class RehomeNode(Node):

    STATE_SCANNING = "SCANNING"
    STATE_APPROACH = "APPROACH"
    STATE_RECORD   = "RECORD"
    STATE_CENTRE   = "CENTRE"
    STATE_ORIENT   = "ORIENT"
    STATE_DONE     = "DONE"

    def __init__(self):
        super().__init__('aruco_rehoming_node')

        # Publishers
        self.cmd_pub    = self.create_publisher(
            Float32MultiArray, '/vision/rehome_cmd', 10)
        self.status_pub = self.create_publisher(String, '/rehome/status', 10)

        # Arducam frame subscription
        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_stamp_ns: int = 0
        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)

        # Camera intrinsics
        self.camera_matrix = ARDUCAM_CAMERA_MATRIX.copy()
        self.dist_coeffs   = ARDUCAM_DIST_COEFFS.copy()

        # Dead-reckoning calibration
        if os.path.exists(DR_CAL_PATH):
            _dr = np.load(DR_CAL_PATH)
            self._mps_per_cmd = float(_dr['ratio'])
            self.get_logger().info(
                f"Dead-reckoning cal loaded: "
                f"{self._mps_per_cmd:.5f} m/s per cmd unit.")
        else:
            self._mps_per_cmd = 0.005
            self.get_logger().warn(
                "Dead-reckoning cal not found — using default 0.005 m/s per cmd unit.")

        # ArUco detector
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        # State machine
        self._state       = self.STATE_SCANNING
        self._target_id: int | None  = None
        self._seen_markers: dict[int, float] = {}

        self._centre_ref_id:         int   = 0
        self._centre_drive_start_ns: int   = 0
        self._centre_drive_dist_m:   float = 0.0

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(f"Rehoming Node ready.  State: {self._state}")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame        = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _publish_wheels(self, left: float, right: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String(); msg.data = text
        self.status_pub.publish(msg)

    def _frame_is_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.last_frame_stamp_ns
        return age_ns < FRAME_TIMEOUT_SEC * 1e9

    def _detect_markers(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        results = []
        if ids is None:
            return results

        half    = MARKER_LENGTH_M / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        for i, marker_id in enumerate(ids.flatten()):
            img_pts = corners[i].reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                results.append((int(marker_id), rvec, tvec.flatten()))
        return results

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._state == self.STATE_DONE:
            self._publish_wheels(0.0, 0.0)
            return

        if not self._frame_is_fresh() or self.latest_frame is None:
            self.get_logger().warn(
                "No fresh Arducam frame — stopping for safety.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        detections = self._detect_markers(self.latest_frame)
        visible: dict[int, tuple] = {d[0]: d for d in detections}

        if self._state == self.STATE_SCANNING:
            self._do_scanning(visible)
        elif self._state == self.STATE_APPROACH:
            self._do_approach(visible)
        elif self._state == self.STATE_RECORD:
            self._do_record(visible)
        elif self._state == self.STATE_CENTRE:
            self._do_centre()
        elif self._state == self.STATE_ORIENT:
            self._do_orient(visible)

    # -- SCANNING ------------------------------------------------------------

    def _do_scanning(self, visible: dict) -> None:
        unseen = [mid for mid in WALL_DISTANCE_TO_CENTER
                  if mid not in self._seen_markers]

        for mid in unseen:
            if mid in visible:
                self._target_id = mid
                self.get_logger().info(f"[SCANNING] Marker {mid} found → APPROACH")
                self._publish_status(f"APPROACH:{mid}")
                self._state = self.STATE_APPROACH
                self._publish_wheels(0.0, 0.0)
                return

        # Only transition to CENTRE when ALL markers are recorded
        if len(self._seen_markers) >= MIN_MARKERS_FOR_CENTRE:
            self.get_logger().info(
                f"[SCANNING] All {MIN_MARKERS_FOR_CENTRE} markers recorded → CENTRE")
            self._begin_centre_phase()
            return

        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

    # -- APPROACH ------------------------------------------------------------

    def _do_approach(self, visible: dict) -> None:
        if self._target_id not in visible:
            self.get_logger().warn(
                f"[APPROACH] Marker {self._target_id} lost — searching.",
                throttle_duration_sec=1.0)
            self._publish_wheels(SCAN_PIVOT_SPEED * 0.5, -SCAN_PIVOT_SPEED * 0.5)
            return

        _, rvec, tvec = visible[self._target_id]
        x_offset = float(tvec[0])
        z_dist   = float(tvec[2])
        error_z  = z_dist - STANDOFF_M

        self.get_logger().info(
            f"[APPROACH] marker={self._target_id}  z={z_dist:.3f}m  "
            f"x={x_offset:.3f}m  err_z={error_z:.3f}m",
            throttle_duration_sec=0.5)

        if abs(error_z) <= STANDOFF_TOL_M:
            self._state = self.STATE_RECORD
            self._publish_wheels(0.0, 0.0)
            return

        if error_z > 0:
            base = APPROACH_SPEED
        else:
            base     = -APPROACH_SPEED
            x_offset = 0.0

        steer   = KP_LATERAL * x_offset
        wl, wr  = base + steer, base - steer
        max_mag = max(abs(wl), abs(wr))
        if max_mag > 100.0:
            wl *= 100.0 / max_mag
            wr *= 100.0 / max_mag

        self._publish_wheels(wl, wr)

    # -- RECORD --------------------------------------------------------------

    def _do_record(self, visible: dict) -> None:
        if self._target_id not in visible:
            self.get_logger().warn(
                f"[RECORD] Marker {self._target_id} not visible — re-approach.")
            self._state = self.STATE_APPROACH
            return

        _, rvec, tvec = visible[self._target_id]
        z_dist = float(tvec[2])

        self._seen_markers[self._target_id] = z_dist
        self.get_logger().info(
            f"[RECORD] Marker {self._target_id} recorded at z={z_dist:.3f}m. "
            f"Total seen: {list(self._seen_markers.keys())}")
        self._publish_status(f"RECORDED:{self._target_id}")

        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info("[RECORD] All 4 markers recorded → CENTRE")
            self._begin_centre_phase()
        else:
            self.get_logger().info(
                f"[RECORD] {len(self._seen_markers)}/4 done → SCANNING")
            self._state = self.STATE_SCANNING
            self._publish_wheels(0.0, 0.0)

    # -- CENTRE --------------------------------------------------------------

    def _begin_centre_phase(self) -> None:
        if not self._seen_markers:
            self.get_logger().error("[CENTRE] No markers — cannot centre.")
            return

        self.get_logger().info("[CENTRE] Per-marker drive estimates:")
        estimates: dict[int, float] = {}
        for mid, z in self._seen_markers.items():
            drive_dist = z - WALL_DISTANCE_TO_CENTER[mid]
            estimates[mid] = drive_dist
            self.get_logger().info(
                f"  Marker {mid}: z={z:.3f}m  "
                f"wall_to_centre={WALL_DISTANCE_TO_CENTER[mid]:.3f}m  "
                f"drive={drive_dist:.3f}m")

        # Use NORTH (0) if available; otherwise pick largest positive estimate
        if 0 in estimates:
            ref_id = 0
        else:
            ref_id = max(estimates, key=lambda k: estimates[k])

        self._centre_drive_dist_m = estimates[ref_id]
        self._centre_ref_id       = ref_id

        self.get_logger().info(
            f"[CENTRE] Reference marker {ref_id}. "
            f"Drive {self._centre_drive_dist_m:.3f} m.")

        if self._centre_drive_dist_m <= 0:
            self.get_logger().warn(
                "[CENTRE] Drive distance ≤ 0 — already past centre. "
                "Skipping to ORIENT.")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        self._state = self.STATE_CENTRE
        self._centre_drive_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("CENTERING")

    def _do_centre(self) -> None:
        if self._centre_drive_dist_m <= CENTRE_ARRIVAL_M:
            self.get_logger().info("[CENTRE] Already at centre → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            return

        elapsed_s       = (self.get_clock().now().nanoseconds
                           - self._centre_drive_start_ns) / 1e9
        distance_driven = elapsed_s * CENTRE_SPEED * self._mps_per_cmd

        self.get_logger().info(
            f"[CENTRE] {distance_driven:.3f} m / {self._centre_drive_dist_m:.3f} m",
            throttle_duration_sec=0.5)

        if distance_driven >= self._centre_drive_dist_m:
            self.get_logger().info("[CENTRE] Centre reached → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
        else:
            self._publish_wheels(CENTRE_SPEED, CENTRE_SPEED)

    # -- ORIENT --------------------------------------------------------------

    def _do_orient(self, visible: dict) -> None:
        if 0 not in visible:
            self.get_logger().info(
                "[ORIENT] NORTH marker not visible — spinning.",
                throttle_duration_sec=1.0)
            self._publish_wheels(-ORIENT_SPEED, ORIENT_SPEED)
            return

        _, rvec, tvec = visible[0]
        x_offset    = float(tvec[0])
        z_dist      = max(float(tvec[2]), 0.1)
        angle_error = math.atan2(x_offset, z_dist)

        self.get_logger().info(
            f"[ORIENT] x_offset={x_offset:.4f}m  angle={math.degrees(angle_error):.1f}°",
            throttle_duration_sec=0.5)

        if abs(angle_error) <= ORIENT_TOL_RAD:
            self.get_logger().info("[ORIENT] Aligned → DONE")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        if angle_error > 0:
            wl, wr =  ORIENT_SPEED, -ORIENT_SPEED
        else:
            wl, wr = -ORIENT_SPEED,  ORIENT_SPEED
        self._publish_wheels(wl, wr)


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