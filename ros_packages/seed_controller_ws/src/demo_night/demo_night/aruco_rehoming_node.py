"""
aruco_rehoming_node.py  —  ArUco rehoming via single pivot scan.

Strategy
--------
SCANNING   Pivot slowly in place.  Each time a new marker comes into view,
           record its bearing (angle from robot centre) and the solvePnP tvec.
           Continue until all 4 markers have been seen.  No driving during scan.

CENTRE     Using the recorded tvecs, compute the 2-D offset from the robot's
           current position to the enclosure centre.  Drive that displacement
           in two legs: rotate to face the centre, then drive straight.

ORIENT     Spin until marker 0 (NORTH) is centred in the frame.

DONE       Stop and publish completion.

Reset      Any time the node receives a /rehome/reset pulse (published by the
           commander when the GUI switches TO rehome mode), the full state
           resets so a fresh run begins.
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

MARKER_LENGTH_M = 0.0854

# ===========================================================================
# Control parameters
# ===========================================================================

SCAN_PIVOT_SPEED  = 15.0   # slow CW spin — minimises motion blur
CENTRE_SPEED      = 38.0   # forward/backward speed during drive-to-centre
ORIENT_SPEED      = 18.0   # pivot speed during final orientation
ROTATE_SPEED      = 20.0   # pivot speed during rotate-to-face-centre leg

ORIENT_TOL_RAD    = math.radians(4.0)   # alignment done threshold
HEADING_TOL_RAD   = math.radians(6.0)   # good-enough heading before driving
CENTRE_ARRIVAL_M  = 0.08                # arrival threshold at centre

FRAME_TIMEOUT_SEC = 1.0

DR_CAL_PATH = (
    "/home/airlab/seed25/ros_packages/seed_controller_ws/src/"
    "demo_night/demo_night/dead_reckoning_cal.npz"
)

# Arducam intrinsics — replace with checkerboard-calibrated values if available
ARDUCAM_CAMERA_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
ARDUCAM_DIST_COEFFS = np.zeros(5, dtype=np.float64)


class RehomeNode(Node):

    STATE_SCANNING = "SCANNING"
    STATE_ROTATING = "ROTATING"   # turning to face computed centre direction
    STATE_CENTRE   = "CENTRE"     # driving straight to centre
    STATE_ORIENT   = "ORIENT"     # final heading alignment to NORTH
    STATE_DONE     = "DONE"

    def __init__(self):
        super().__init__('aruco_rehoming_node')

        # Publishers
        self.cmd_pub    = self.create_publisher(
            Float32MultiArray, '/vision/rehome_cmd', 10)
        self.status_pub = self.create_publisher(String, '/rehome/status', 10)

        # Arducam subscription
        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_stamp_ns: int = 0
        self.create_subscription(
            Image, '/vision/arducam_raw', self._frame_cb, SENSOR_QOS)

        # Reset subscription — commander publishes here when REHOME is activated
        self.create_subscription(
            String, '/rehome/reset', self._reset_cb, 10)

        self.camera_matrix = ARDUCAM_CAMERA_MATRIX.copy()
        self.dist_coeffs   = ARDUCAM_DIST_COEFFS.copy()

        # Dead-reckoning speed ratio
        if os.path.exists(DR_CAL_PATH):
            _dr = np.load(DR_CAL_PATH)
            self._mps_per_cmd = float(_dr['ratio'])
            self.get_logger().info(
                f"Dead-reckoning cal loaded: {self._mps_per_cmd:.5f} m/s per cmd unit.")
        else:
            self._mps_per_cmd = 0.005
            self.get_logger().warn(
                "Dead-reckoning cal not found — using default 0.005 m/s per cmd.")

        # ArUco detector
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        self._init_state()

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(f"Rehoming Node ready.  State: {self._state}")

    # -----------------------------------------------------------------------
    # State initialisation / reset
    # -----------------------------------------------------------------------

    def _init_state(self) -> None:
        """Fully reset the state machine.  Called on startup and on reset signal."""
        self._state = self.STATE_SCANNING

        # marker_id → tvec (x, y, z) recorded during the pivot scan
        self._seen_markers: dict[int, np.ndarray] = {}

        # Centre drive parameters computed after scan
        self._centre_angle_rad:      float = 0.0   # robot must rotate to face this
        self._centre_dist_m:         float = 0.0   # then drive this far
        self._centre_drive_start_ns: int   = 0
        self._rotate_start_ns:       int   = 0
        self._rotate_duration_s:     float = 0.0

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _frame_cb(self, msg: Image) -> None:
        self.latest_frame        = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    def _reset_cb(self, msg: String) -> None:
        self.get_logger().info("[REHOME] Reset received — restarting scan.")
        self._init_state()
        self._publish_status("RESET")

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

    def _detect_markers(self, frame: np.ndarray) -> dict[int, np.ndarray]:
        """Returns dict of marker_id → tvec for all detected markers."""
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
            if int(marker_id) not in WALL_DISTANCE_TO_CENTER:
                continue   # ignore unknown marker IDs
            img_pts = corners[i].reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                results[int(marker_id)] = tvec.flatten()
        return results

    # -----------------------------------------------------------------------
    # Centre computation
    # -----------------------------------------------------------------------

    def _compute_centre_offset(self) -> tuple[float, float]:
        """
        Given the tvecs recorded for each wall marker, compute the 2-D vector
        (dx, dy) from the robot's current position to the enclosure centre,
        in the robot's camera frame (x = right, z = forward).

        For each marker we know:
          - tvec[0] = lateral offset of marker from camera (+ = right)
          - tvec[2] = depth of marker from camera (always +)
          - WALL_DISTANCE_TO_CENTER[id] = distance from that marker to centre

        The marker sits on the wall directly ahead (in 3-D).  The unit vector
        FROM robot TO marker in robot frame is:
            u = (tvec[0], tvec[2]) / |tvec|_xy  (2-D, ignoring height)

        The centre lies on the opposite side of that wall at distance
        WALL_DISTANCE_TO_CENTER from the marker, i.e.:
            pos_centre = pos_marker - wall_to_centre * u_wall_normal

        In robot frame the wall normal pointing INTO the enclosure is -u
        (pointing back toward the robot from the wall).  So:
            centre_vec = tvec_xy - WALL_DISTANCE_TO_CENTER * u_xy_normalised

        We average across all recorded markers for the best estimate.
        """
        dx_list: list[float] = []
        dz_list: list[float] = []

        for mid, tvec in self._seen_markers.items():
            mx = float(tvec[0])   # lateral
            mz = float(tvec[2])   # depth
            dist_marker = math.sqrt(mx**2 + mz**2)
            if dist_marker < 0.01:
                continue

            # Unit vector from robot toward marker
            ux = mx / dist_marker
            uz = mz / dist_marker

            wall_to_centre = WALL_DISTANCE_TO_CENTER[mid]

            # Vector from robot to centre via this marker
            cx = mx - wall_to_centre * ux
            cz = mz - wall_to_centre * uz

            dx_list.append(cx)
            dz_list.append(cz)

            self.get_logger().info(
                f"  Marker {mid}: tvec=({mx:.3f}, {mz:.3f})  "
                f"centre_vec=({cx:.3f}, {cz:.3f})")

        if not dx_list:
            return 0.0, 0.0

        return float(np.mean(dx_list)), float(np.mean(dz_list))

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

        visible = self._detect_markers(self.latest_frame)

        if self._state == self.STATE_SCANNING:
            self._do_scanning(visible)
        elif self._state == self.STATE_ROTATING:
            self._do_rotating(visible)
        elif self._state == self.STATE_CENTRE:
            self._do_centre()
        elif self._state == self.STATE_ORIENT:
            self._do_orient(visible)

    # -- SCANNING ------------------------------------------------------------

    def _do_scanning(self, visible: dict[int, np.ndarray]) -> None:
        # Record any newly visible markers
        for mid, tvec in visible.items():
            if mid not in self._seen_markers:
                self._seen_markers[mid] = tvec
                self.get_logger().info(
                    f"[SCANNING] Marker {mid} recorded: "
                    f"x={tvec[0]:.3f}m  z={tvec[2]:.3f}m  "
                    f"({len(self._seen_markers)}/4 total)")
                self._publish_status(f"FOUND:{mid}")

        # Once all 4 are seen, compute centre and transition
        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info(
                "[SCANNING] All 4 markers recorded — computing centre.")
            self._publish_wheels(0.0, 0.0)
            self._begin_centre_phase()
            return

        # Keep pivoting CW slowly
        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

    # -- CENTRE (two-leg: rotate then drive) ---------------------------------

    def _begin_centre_phase(self) -> None:
        dx, dz = self._compute_centre_offset()
        dist   = math.sqrt(dx**2 + dz**2)

        if dist < CENTRE_ARRIVAL_M:
            self.get_logger().info(
                "[CENTRE] Already at centre — skipping to ORIENT.")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        # Angle to centre in robot frame: atan2(lateral, forward)
        # Positive angle = centre is to the RIGHT → must turn CW
        angle_to_centre = math.atan2(dx, dz)

        self.get_logger().info(
            f"[CENTRE] Centre offset: dx={dx:.3f}m  dz={dz:.3f}m  "
            f"dist={dist:.3f}m  angle={math.degrees(angle_to_centre):.1f}°")

        self._centre_angle_rad = angle_to_centre
        self._centre_dist_m    = dist

        # Estimate rotation duration from angle and pivot speed
        # angular_velocity ≈ 2 * wheel_speed * mps_per_cmd / wheel_base
        # Use a fixed wheel_base estimate of 0.40 m
        WHEEL_BASE = 0.40
        angular_vel_rps = (2.0 * ROTATE_SPEED * self._mps_per_cmd) / WHEEL_BASE
        self._rotate_duration_s = abs(angle_to_centre) / angular_vel_rps

        self.get_logger().info(
            f"[CENTRE] Rotating {math.degrees(angle_to_centre):.1f}° "
            f"({self._rotate_duration_s:.2f}s), then driving {dist:.3f}m.")

        self._state = self.STATE_ROTATING
        self._rotate_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("CENTERING")

    def _do_rotating(self, visible: dict[int, np.ndarray]) -> None:
        """Rotate in place to face the computed centre direction."""
        elapsed_s = (self.get_clock().now().nanoseconds
                     - self._rotate_start_ns) / 1e9

        if elapsed_s >= self._rotate_duration_s:
            self.get_logger().info(
                "[CENTRE] Rotation complete — driving to centre.")
            self._publish_wheels(0.0, 0.0)
            self._centre_drive_start_ns = self.get_clock().now().nanoseconds
            self._state = self.STATE_CENTRE
            return

        # CW if angle positive (centre to right), CCW if negative (centre to left)
        if self._centre_angle_rad >= 0:
            self._publish_wheels(ROTATE_SPEED, -ROTATE_SPEED)   # CW
        else:
            self._publish_wheels(-ROTATE_SPEED, ROTATE_SPEED)   # CCW

    def _do_centre(self) -> None:
        """Drive straight toward the centre for the calculated distance."""
        elapsed_s       = (self.get_clock().now().nanoseconds
                           - self._centre_drive_start_ns) / 1e9
        distance_driven = elapsed_s * CENTRE_SPEED * self._mps_per_cmd

        self.get_logger().info(
            f"[CENTRE] Driven {distance_driven:.3f}m / {self._centre_dist_m:.3f}m",
            throttle_duration_sec=0.5)

        if distance_driven >= self._centre_dist_m:
            self.get_logger().info("[CENTRE] Centre reached → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
        else:
            self._publish_wheels(CENTRE_SPEED, CENTRE_SPEED)

    # -- ORIENT --------------------------------------------------------------

    def _do_orient(self, visible: dict[int, np.ndarray]) -> None:
        if 0 not in visible:
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
            self.get_logger().info("[ORIENT] Aligned → DONE")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        if angle_error > 0:
            self._publish_wheels( ORIENT_SPEED, -ORIENT_SPEED)   # CW
        else:
            self._publish_wheels(-ORIENT_SPEED,  ORIENT_SPEED)   # CCW


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