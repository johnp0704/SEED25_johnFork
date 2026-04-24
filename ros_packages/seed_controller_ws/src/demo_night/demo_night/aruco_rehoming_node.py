"""
aruco_rehoming_node.py  —  ArUco rehoming via single pivot scan.

Strategy
--------
IDLE       The node does nothing and publishes no commands.  This is the
           startup state.  The node waits here until the commander sends a
           /rehome/reset pulse (which happens automatically whenever the GUI
           switches to REHOME mode).

SCANNING   Pivot slowly CW in place.  Each time a new marker comes into view,
           record its tvec.  Continue until all 4 markers have been seen.
           No driving during the scan.

ROTATING   Turn in place to face the computed direction of the enclosure centre.

CENTRE     Drive straight toward the centre for the calculated distance.

ORIENT     Spin until marker 0 (NORTH) is centred in the frame.

DONE       Stop and publish completion status.

Root cause of the previous "robot does not move" bug
-----------------------------------------------------
The node previously started in SCANNING on startup and immediately began
publishing pivot commands to /vision/rehome_cmd.  The commander only routes
that topic to the motors while current_mode == "REHOME".  Because the node
was already mid-scan before the user clicked REHOME in the GUI, all early
commands were silently discarded.  By the time the commander entered REHOME
mode, the scan had already recorded some markers but the robot had never
actually pivoted — so the scan data was meaningless (the tvec values were
captured from whatever pose the robot happened to be in, not from a
controlled pivot).

The fix: start in IDLE, publish nothing, and only begin the pivot scan
after receiving the /rehome/reset signal that the commander emits the
moment the GUI switches to REHOME mode.
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
# Physical constants — measure these for your enclosure
# ===========================================================================

# Distance from each wall marker face to the enclosure geometric centre (m).
WALL_DISTANCE_TO_CENTER = {
    0: 2.1336,    # NORTH
    1: 1.3208,    # EAST
    2: 1.81769,   # SOUTH
    3: 0.9017,    # WEST
}

# Printed marker side length (m).  Must match the physical printout exactly.
MARKER_LENGTH_M = 0.0854

# ===========================================================================
# Control parameters
# ===========================================================================

SCAN_PIVOT_SPEED = 15.0   # slow CW spin during SCANNING (cmd units)
CENTRE_SPEED     = 38.0   # forward speed during CENTRE leg (cmd units)
ORIENT_SPEED     = 18.0   # pivot speed during final ORIENT (cmd units)
ROTATE_SPEED     = 20.0   # pivot speed during ROTATING leg (cmd units)

ORIENT_TOL_RAD   = math.radians(4.0)   # heading alignment done threshold
CENTRE_ARRIVAL_M = 0.08                # stop driving when within this of centre (m)

FRAME_TIMEOUT_SEC = 1.0   # declare frame stale after this long (s)

WHEEL_BASE_M = 0.40       # robot wheel-base estimate for rotation timing (m)

# Path to the dead-reckoning calibration file.
DR_CAL_PATH = (
    "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
    "demo_night/demo_night/dead_reckoning_cal.npz"
)

# Arducam intrinsics — replace with checkerboard-calibrated values if available.
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

        # --- Publishers ---
        self.cmd_pub    = self.create_publisher(
            Float32MultiArray, '/vision/rehome_cmd', 10)
        self.status_pub = self.create_publisher(String, '/rehome/status', 10)

        # --- Arducam subscription ---
        self.bridge                       = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_stamp_ns: int        = 0
        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)

        # --- Reset / start signal (emitted by commander on REHOME entry) ---
        self.create_subscription(
            String, '/rehome/reset', self._reset_cb, 10)

        # --- Camera model ---
        self.camera_matrix = ARDUCAM_CAMERA_MATRIX.copy()
        self.dist_coeffs   = ARDUCAM_DIST_COEFFS.copy()

        # --- Dead-reckoning speed ratio ---
        if os.path.exists(DR_CAL_PATH):
            _dr = np.load(DR_CAL_PATH)
            self._mps_per_cmd = float(_dr['ratio'])
            self.get_logger().info(
                f"Dead-reckoning cal loaded: "
                f"{self._mps_per_cmd:.5f} m/s per cmd unit.")
        else:
            self._mps_per_cmd = 0.005
            self.get_logger().warn(
                "Dead-reckoning cal not found — using default 0.005 m/s per cmd.")

        # --- ArUco detector ---
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        # --- State machine — start IDLE, wait for reset signal ---
        self._state = self.STATE_IDLE
        self._clear_scan_data()

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            "Rehoming Node ready.  Waiting for /rehome/reset to begin scan.")

    # =======================================================================
    # Helpers
    # =======================================================================

    def _clear_scan_data(self) -> None:
        self._seen_markers: dict[int, np.ndarray] = {}
        self._centre_angle_rad:      float = 0.0
        self._centre_dist_m:         float = 0.0
        self._rotate_duration_s:     float = 0.0
        self._rotate_start_ns:       int   = 0
        self._centre_drive_start_ns: int   = 0

    def _publish_wheels(self, left: float, right: float) -> None:
        msg      = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _frame_is_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.last_frame_stamp_ns
        return age_ns < FRAME_TIMEOUT_SEC * 1e9

    # =======================================================================
    # Callbacks
    # =======================================================================

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame        = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    def _reset_cb(self, msg: String) -> None:
        """
        Called by the commander the moment the GUI switches to REHOME mode.
        Wipes any previous scan data and kicks off a fresh pivot scan.
        The commander has already set current_mode = "REHOME" before publishing
        this signal, so our wheel commands will be routed to the motors
        immediately.
        """
        self.get_logger().info("[REHOME] Reset received — starting pivot scan.")
        self._clear_scan_data()
        self._state = self.STATE_SCANNING
        self._publish_status("SCANNING")

    # =======================================================================
    # ArUco detection
    # =======================================================================

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

        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
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

    # =======================================================================
    # Centre offset computation
    # =======================================================================

    def _compute_centre_offset(self) -> tuple[float, float]:
        """
        For each recorded wall marker, project the enclosure centre into the
        robot's camera frame, then average across all markers.

        Camera frame: x = right (+), z = forward (+).
        For a marker at tvec = (mx, *, mz):
            unit vector toward marker: u = (mx, mz) / |(mx, mz)|
            centre lies at:           centre_vec = (mx, mz) - d * u
        where d = WALL_DISTANCE_TO_CENTER[marker_id].
        """
        dx_list: list[float] = []
        dz_list: list[float] = []

        for mid, tvec in self._seen_markers.items():
            mx   = float(tvec[0])
            mz   = float(tvec[2])
            dist = math.sqrt(mx**2 + mz**2)
            if dist < 0.01:
                continue

            ux  = mx / dist
            uz  = mz / dist
            d   = WALL_DISTANCE_TO_CENTER[mid]
            cx  = mx - d * ux
            cz  = mz - d * uz

            dx_list.append(cx)
            dz_list.append(cz)

            self.get_logger().info(
                f"  Marker {mid}: tvec=({mx:.3f}, {mz:.3f})m  "
                f"→ centre_vec=({cx:.3f}, {cz:.3f})m")

        if not dx_list:
            return 0.0, 0.0
        return float(np.mean(dx_list)), float(np.mean(dz_list))

    # =======================================================================
    # State machine
    # =======================================================================

    def _control_loop(self) -> None:

        # IDLE — publish nothing, wait for reset signal.
        if self._state == self.STATE_IDLE:
            return

        # DONE — hold motors stopped.
        if self._state == self.STATE_DONE:
            self._publish_wheels(0.0, 0.0)
            return

        # All active states need a fresh camera frame.
        if not self._frame_is_fresh() or self.latest_frame is None:
            self.get_logger().warn(
                "No fresh Arducam frame — stopping for safety.",
                throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        visible = self._detect_markers(self.latest_frame)

        if   self._state == self.STATE_SCANNING:
            self._do_scanning(visible)
        elif self._state == self.STATE_ROTATING:
            self._do_rotating()
        elif self._state == self.STATE_CENTRE:
            self._do_centre()
        elif self._state == self.STATE_ORIENT:
            self._do_orient(visible)

    # -----------------------------------------------------------------------
    # SCANNING — slow CW pivot, record each new marker once
    # -----------------------------------------------------------------------

    def _do_scanning(self, visible: dict[int, np.ndarray]) -> None:
        for mid, tvec in visible.items():
            if mid not in self._seen_markers:
                self._seen_markers[mid] = tvec
                self.get_logger().info(
                    f"[SCANNING] Marker {mid} recorded: "
                    f"x={tvec[0]:.3f}m  z={tvec[2]:.3f}m  "
                    f"({len(self._seen_markers)}/4 total)")
                self._publish_status(f"FOUND:{mid}")

        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info(
                "[SCANNING] All 4 markers recorded — computing centre.")
            self._publish_wheels(0.0, 0.0)
            self._begin_rotating()
            return

        # CW pivot: left forward, right backward
        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

    # -----------------------------------------------------------------------
    # ROTATING — turn to face the computed centre direction
    # -----------------------------------------------------------------------

    def _begin_rotating(self) -> None:
        dx, dz = self._compute_centre_offset()
        dist   = math.sqrt(dx**2 + dz**2)

        if dist < CENTRE_ARRIVAL_M:
            self.get_logger().info(
                "[ROTATING] Already at centre — skipping to ORIENT.")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        # atan2(lateral, forward): positive = centre is to the RIGHT
        angle = math.atan2(dx, dz)

        self.get_logger().info(
            f"[ROTATING] Centre: dx={dx:.3f}m  dz={dz:.3f}m  "
            f"dist={dist:.3f}m  angle={math.degrees(angle):.1f}°")

        self._centre_angle_rad = angle
        self._centre_dist_m    = dist

        # Time to rotate: t = |angle| / ω,  ω = (2 × v_wheel) / wheelbase
        omega = (2.0 * ROTATE_SPEED * self._mps_per_cmd) / WHEEL_BASE_M
        self._rotate_duration_s = abs(angle) / omega if omega > 0 else 0.0

        self.get_logger().info(
            f"[ROTATING] Rotating {math.degrees(angle):.1f}° "
            f"for {self._rotate_duration_s:.2f}s, "
            f"then driving {dist:.3f}m.")

        self._state           = self.STATE_ROTATING
        self._rotate_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("ROTATING")

    def _do_rotating(self) -> None:
        elapsed_s = (self.get_clock().now().nanoseconds
                     - self._rotate_start_ns) / 1e9

        if elapsed_s >= self._rotate_duration_s:
            self.get_logger().info("[ROTATING] Complete — driving to centre.")
            self._publish_wheels(0.0, 0.0)
            self._centre_drive_start_ns = self.get_clock().now().nanoseconds
            self._state = self.STATE_CENTRE
            self._publish_status("CENTERING")
            return

        # CW (angle > 0): left fwd, right back
        # CCW (angle < 0): left back, right fwd
        if self._centre_angle_rad >= 0:
            self._publish_wheels( ROTATE_SPEED, -ROTATE_SPEED)
        else:
            self._publish_wheels(-ROTATE_SPEED,  ROTATE_SPEED)

    # -----------------------------------------------------------------------
    # CENTRE — drive straight toward the centre
    # -----------------------------------------------------------------------

    def _do_centre(self) -> None:
        elapsed_s       = (self.get_clock().now().nanoseconds
                           - self._centre_drive_start_ns) / 1e9
        distance_driven = elapsed_s * CENTRE_SPEED * self._mps_per_cmd

        self.get_logger().info(
            f"[CENTRE] Driven {distance_driven:.3f}m / {self._centre_dist_m:.3f}m",
            throttle_duration_sec=0.5)

        if distance_driven >= self._centre_dist_m:
            self.get_logger().info("[CENTRE] Arrived — starting ORIENT.")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
        else:
            self._publish_wheels(CENTRE_SPEED, CENTRE_SPEED)

    # -----------------------------------------------------------------------
    # ORIENT — align heading to face NORTH (marker ID 0)
    # -----------------------------------------------------------------------

    def _do_orient(self, visible: dict[int, np.ndarray]) -> None:
        if 0 not in visible:
            # Spin CCW to search: left back, right forward
            self.get_logger().info(
                "[ORIENT] NORTH marker not visible — spinning CCW.",
                throttle_duration_sec=1.0)
            self._publish_wheels(-ORIENT_SPEED, ORIENT_SPEED)
            return

        tvec        = visible[0]
        x_offset    = float(tvec[0])
        z_dist      = max(float(tvec[2]), 0.1)
        angle_error = math.atan2(x_offset, z_dist)

        self.get_logger().info(
            f"[ORIENT] x_offset={x_offset:.4f}m  "
            f"angle={math.degrees(angle_error):.1f}°",
            throttle_duration_sec=0.5)

        if abs(angle_error) <= ORIENT_TOL_RAD:
            self.get_logger().info("[ORIENT] Aligned — DONE.")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        # Positive angle → marker to the right → CW to face it
        # CW:  left fwd (+), right back (-)
        # CCW: left back (-), right fwd (+)
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