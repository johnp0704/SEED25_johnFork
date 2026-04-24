"""
aruco_rehoming_node.py  —  ArUco rehoming via single pivot scan.

Strategy
--------
IDLE       Publish nothing.  Wait for /rehome/reset from the commander.

SCANNING   Pivot CW in place.  A mandatory PIVOT_DWELL_SEC window at the
           start of the scan lets the Sabertooth ramp up and the robot
           physically move before any marker is accepted as "seen".  After
           the dwell, each newly visible marker is recorded once.  The pivot
           continues until all 4 markers have been logged.

ROTATING   Turn in place to face the computed enclosure centre direction.

CENTRE     Drive straight toward the centre for the dead-reckoned distance.

ORIENT     Spin until marker 0 (NORTH) is centred in the frame.

DONE       Hold motors stopped, publish "DONE" status.

Why the motors were only briefly actuating
------------------------------------------
Two compounding issues:

1. Loop rate mismatch.
   The rehoming node ran at 10 Hz; the commander runs at 20 Hz.  The
   commander's _safe_cmd timeout is 1.0 s, so that alone was not the
   problem.  However, the Sabertooth trapezoidal ramp with accel_rate=80
   and call_rate_hz=20 gives an accel step of 4 units/call.  Reaching
   SCAN_PIVOT_SPEED=15 requires ~4 commander calls = 200 ms of sustained
   commands.

2. Immediate marker detection aborting the scan.
   If marker 0 was visible from the robot's startup pose, the SCANNING
   handler recorded it on its very first tick (before the robot had moved
   at all) and then — if all 4 happened to be in view — immediately
   transitioned to ROTATING and published [0,0].  The motors heard
   "spin up" for 100 ms then "stop", producing exactly the brief
   actuation sound observed.

Fix: run the control loop at 20 Hz (matching the commander and Sabertooth
call rate) and enforce a PIVOT_DWELL_SEC mandatory spin window at the
start of SCANNING during which no markers are recorded.  This guarantees
the robot is physically moving and the Sabertooth has fully ramped up
before any detection can trigger a state transition.
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

WALL_DISTANCE_TO_CENTER: dict[int, float] = {
    0: 2.1336,     # NORTH
    1: 1.3208,     # EAST
    2: 1.81769,    # SOUTH
    3: 0.9017,     # WEST
}

MARKER_LENGTH_M = 0.0854   # printed marker side length (m)

# ===========================================================================
# Control / timing parameters
# ===========================================================================

LOOP_HZ          = 20.0   # must match commander SABERTOOTH_CALL_RATE_HZ

# Mandatory spin window at the start of SCANNING before any marker is
# accepted.  Gives the Sabertooth time to ramp up and the robot time to
# physically move away from its startup pose.
# At accel_rate=80, call_rate=20 Hz: step = 4 units/call.
# Reaching speed 15: ceil(15/4) = 4 calls = 0.2 s.
# 0.75 s is generous and ensures the robot has rotated a visible amount.
PIVOT_DWELL_SEC  = 0.75

SCAN_PIVOT_SPEED = 15.0   # CW spin speed during SCANNING (cmd units)
ROTATE_SPEED     = 20.0   # pivot speed during ROTATING  (cmd units)
CENTRE_SPEED     = 38.0   # forward speed during CENTRE  (cmd units)
ORIENT_SPEED     = 18.0   # pivot speed during ORIENT    (cmd units)

ORIENT_TOL_RAD   = math.radians(4.0)
CENTRE_ARRIVAL_M = 0.08    # metres — skip drive if already this close to centre

FRAME_TIMEOUT_SEC = 1.0    # declare frame stale after this long (s)
WHEEL_BASE_M      = 0.40   # wheel-base for rotation time estimate (m)

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
                f"Dead-reckoning cal loaded: "
                f"{self._mps_per_cmd:.5f} m/s per cmd unit.")
        else:
            self._mps_per_cmd = 0.005
            self.get_logger().warn(
                "Dead-reckoning cal not found — using default 0.005 m/s per cmd.")

        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        self._state = self.STATE_IDLE
        self._clear_scan_data()

        # 20 Hz loop — matches commander and Sabertooth call rate
        self.create_timer(1.0 / LOOP_HZ, self._control_loop)
        self.get_logger().info(
            "Rehoming Node ready.  Waiting for /rehome/reset to begin scan.")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _clear_scan_data(self) -> None:
        self._seen_markers:          dict[int, np.ndarray] = {}
        self._scan_start_ns:         int   = 0    # when SCANNING began
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
        """
        Emitted by the commander the instant current_mode becomes "REHOME".
        The commander has already switched routing before publishing this,
        so our very next wheel command will reach the motors.
        """
        self.get_logger().info("[REHOME] Reset — starting pivot scan.")
        self._clear_scan_data()
        self._state        = self.STATE_SCANNING
        self._scan_start_ns = self.get_clock().now().nanoseconds
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
    # Centre offset
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
                f"  Marker {mid}: tvec=({mx:.3f}, {mz:.3f})m  "
                f"→ centre=({cx:.3f}, {cz:.3f})m")

        if not dx_list:
            return 0.0, 0.0
        return float(np.mean(dx_list)), float(np.mean(dz_list))

    # -----------------------------------------------------------------------
    # Main control loop — 20 Hz
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:

        if self._state == self.STATE_IDLE:
            return   # publish nothing

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

        if   self._state == self.STATE_SCANNING:
            self._do_scanning(visible)
        elif self._state == self.STATE_ROTATING:
            self._do_rotating()
        elif self._state == self.STATE_CENTRE:
            self._do_centre()
        elif self._state == self.STATE_ORIENT:
            self._do_orient(visible)

    # -----------------------------------------------------------------------
    # SCANNING
    # -----------------------------------------------------------------------

    def _do_scanning(self, visible: dict[int, np.ndarray]) -> None:
        now_ns    = self.get_clock().now().nanoseconds
        dwell_sec = (now_ns - self._scan_start_ns) / 1e9

        # Always publish the pivot command first — this keeps the Sabertooth
        # ramp fed every loop tick regardless of detection state.
        # CW: left forward, right backward.
        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

        # Do not accept any detections until the dwell window has elapsed.
        # This guarantees the robot has physically moved before we record
        # the angular position of each marker.
        if dwell_sec < PIVOT_DWELL_SEC:
            self.get_logger().info(
                f"[SCANNING] Dwell: {dwell_sec:.2f}s / {PIVOT_DWELL_SEC:.2f}s "
                "— not recording yet.",
                throttle_duration_sec=0.5)
            return

        # Record newly visible markers
        for mid, tvec in visible.items():
            if mid not in self._seen_markers:
                self._seen_markers[mid] = tvec
                self.get_logger().info(
                    f"[SCANNING] Marker {mid} recorded: "
                    f"x={tvec[0]:.3f}m  z={tvec[2]:.3f}m  "
                    f"({len(self._seen_markers)}/4 total)")
                self._publish_status(f"FOUND:{mid}")

        # All four seen — transition (stop is published by _begin_rotating)
        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info(
                "[SCANNING] All 4 markers recorded — computing centre.")
            self._begin_rotating()

    # -----------------------------------------------------------------------
    # ROTATING
    # -----------------------------------------------------------------------

    def _begin_rotating(self) -> None:
        self._publish_wheels(0.0, 0.0)   # stop the scan pivot first

        dx, dz = self._compute_centre_offset()
        dist   = math.sqrt(dx**2 + dz**2)

        if dist < CENTRE_ARRIVAL_M:
            self.get_logger().info(
                "[ROTATING] Already at centre — skipping to ORIENT.")
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
            return

        angle = math.atan2(dx, dz)   # positive = centre is to the RIGHT

        self.get_logger().info(
            f"[ROTATING] Centre: dx={dx:.3f}m  dz={dz:.3f}m  "
            f"dist={dist:.3f}m  angle={math.degrees(angle):.1f}°")

        self._centre_angle_rad = angle
        self._centre_dist_m    = dist

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

        # CW (angle >= 0): left fwd, right back
        # CCW (angle < 0): left back, right fwd
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
    # ORIENT
    # -----------------------------------------------------------------------

    def _do_orient(self, visible: dict[int, np.ndarray]) -> None:
        if 0 not in visible:
            self.get_logger().info(
                "[ORIENT] NORTH marker not visible — spinning CCW.",
                throttle_duration_sec=1.0)
            # CCW: left back, right forward
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

        # CW (marker right): left fwd, right back
        # CCW (marker left): left back, right fwd
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