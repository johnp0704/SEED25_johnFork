"""
aruco_rehoming_node.py

Full rehoming state machine using four ArUco wall markers.

Physical setup
--------------
Each of the four walls of the rectangular enclosure has one printed ArUco
marker (DICT_4X4_50) mounted at a known, fixed height.  The IDs are assigned
as follows (edit WALL_MAP to match your physical layout):

    ID 0 → NORTH wall   (the "home" heading — robot will face this at the end)
    ID 1 → EAST  wall
    ID 2 → SOUTH wall
    ID 3 → WEST  wall

WALL_DISTANCE_TO_CENTER (metres) stores how far each marker is from the
geometric centre of the enclosure.  Measure these with a tape measure and
fill them in before running.

State machine
-------------
SCANNING        Rotate slowly until any marker is found.
APPROACH        Drive toward the currently-visible marker until we are at the
                desired stand-off distance (STANDOFF_M).
RECORD          Grab tvec, compute how far we still need to travel to reach
                the centre based on the known geometry, store the result.
                Transition → SCANNING for the next marker (or → CENTRE when
                all four are seen, or enough to compute a good estimate).
CENTRE          Drive to the computed enclosure centre using the averaged
                dead-reckoning estimate.
ORIENT          Spin in place until marker ID 0 (NORTH) is directly ahead
                (x-offset ≈ 0).
DONE            Publish zero commands.  Emit a completion signal.

Topics
------
Subscribes : /vision/realsense_color   (sensor_msgs/Image)  — from realsense_node
Publishes  : /vision/rehome_cmd        (std_msgs/Float32MultiArray)
             /rehome/status            (std_msgs/String)
"""
from __future__ import annotations
import math
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
import cv2.aruco as aruco
import numpy as np


# ===========================================================================
# Physical constants — EDIT THESE to match your enclosure
# ===========================================================================

# Distance from each wall marker to the geometric centre of the enclosure (m).
# Key = ArUco marker ID, Value = distance in metres.
WALL_DISTANCE_TO_CENTER = {
    0: 2.1336,   # NORTH wall marker → centre
    1: 1.3208,   # EAST  wall marker → centre
    2: 1.81769,   # SOUTH wall marker → centre
    3: 0.9017,   # WEST  wall marker → centre
}

# Compass bearing (radians, robot-frame) of each wall marker relative to the
# enclosure centre, measured when the robot is centred and facing NORTH.
# NORTH = 0, EAST = -π/2, SOUTH = π, WEST = π/2
WALL_BEARING_FROM_CENTER = {
    0:  0.0,            # NORTH
    1: -math.pi / 2,    # EAST
    2:  math.pi,        # SOUTH
    3:  math.pi / 2,    # WEST
}

# Physical size of the printed marker side (metres).  Must match print.
MARKER_LENGTH_M = 0.0854

# ===========================================================================
# Control parameters
# ===========================================================================

# Stand-off distance for the APPROACH → RECORD transition (m).
STANDOFF_M       = 0.80   # Stop this far from the wall marker to get a pose
STANDOFF_TOL_M   = 0.05   # ± tolerance

# How close to the centre we need to be to call it done (m).
CENTRE_ARRIVAL_M = 0.10

# Heading error (rad) below which we call orientation complete.
ORIENT_TOL_RAD   = math.radians(5.0)

# Speeds (Sabertooth command units, 0-100)
SCAN_PIVOT_SPEED = 25.0   # slow spin during SCANNING
APPROACH_SPEED   = 35.0   # forward during APPROACH
CENTRE_SPEED     = 38.0   # forward during CENTRE
ORIENT_SPEED     = 28.0   # pivot during ORIENT

# PID-like lateral correction gain during APPROACH
KP_LATERAL       = 40.0

# Minimum number of unique marker IDs we must see before attempting to centre.
# Set to 2 for a minimal viable estimate; 4 for full accuracy.
MIN_MARKERS_FOR_CENTRE = 2

# Maximum frame age before we declare the marker lost (seconds)
FRAME_TIMEOUT_SEC = 0.5

# ===========================================================================


class RehomeNode(Node):

    # -----------------------------------------------------------------------
    # States
    # -----------------------------------------------------------------------
    STATE_SCANNING  = "SCANNING"
    STATE_APPROACH  = "APPROACH"
    STATE_RECORD    = "RECORD"
    STATE_CENTRE    = "CENTRE"
    STATE_ORIENT    = "ORIENT"
    STATE_DONE      = "DONE"

    def __init__(self):
        super().__init__('rehome_node')

        # Publishers
        self.cmd_pub    = self.create_publisher(Float32MultiArray, '/vision/rehome_cmd', 10)
        self.status_pub = self.create_publisher(String, '/rehome/status', 10)

        # Subscriber — receives colour frames from the central realsense_node
        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.last_frame_stamp_ns: int = 0
        self.create_subscription(Image, '/vision/realsense_color',
                                 self._color_cb, SENSOR_QOS)

        # Camera intrinsics — will be populated from the CameraInfo message.
        # We also accept a latched /vision/realsense_camera_info if available.
        # Fallback: reasonable RealSense D435 defaults (metres / pixels).
        self.camera_matrix = np.array([
            [615.0,   0.0, 320.0],
            [  0.0, 615.0, 240.0],
            [  0.0,   0.0,   1.0],
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
        self._intrinsics_loaded = False

        try:
            from sensor_msgs.msg import CameraInfo
            self.create_subscription(
                CameraInfo, '/vision/realsense_camera_info',
                self._camera_info_cb, 1)
        except Exception:
            pass  # CameraInfo subscription is optional

        # ArUco detector (OpenCV 4.7+ API)
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters(),
        )

        # State machine
        self._state            = self.STATE_SCANNING
        self._target_id: int | None = None   # marker ID we are currently approaching
        self._seen_markers: dict[int, float] = {}  # id → measured z distance at STANDOFF

        # Dead-reckoning counters for CENTRE phase
        self._centre_drive_start_ns: int = 0
        self._centre_drive_dist_m:   float = 0.0

        # Twist accumulator for current heading estimate (radians, relative to
        # initial pose).  We track this so we know our heading during ORIENT.
        self._heading_rad: float = 0.0

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"Rehoming Node ready.  State: {self._state}"
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _color_cb(self, msg: Image) -> None:
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame_stamp_ns = self.get_clock().now().nanoseconds

    def _camera_info_cb(self, msg) -> None:
        if self._intrinsics_loaded:
            return
        k = msg.k  # row-major 3×3
        self.camera_matrix = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._intrinsics_loaded = True
        self.get_logger().info("Camera intrinsics loaded from CameraInfo topic.")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _publish_wheels(self, left: float, right: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _detect_markers(self, frame: np.ndarray):
        """
        Returns a list of (marker_id, rvec, tvec) for all detected markers.
        Uses solvePnP directly (avoids deprecated estimatePoseSingleMarkers).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []
        if ids is None:
            return results

        # 3-D coordinates of marker corners in marker frame (z forward)
        half = MARKER_LENGTH_M / 2.0
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

    def _frame_is_fresh(self) -> bool:
        age_ns = self.get_clock().now().nanoseconds - self.last_frame_stamp_ns
        return age_ns < FRAME_TIMEOUT_SEC * 1e9

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------

    def _control_loop(self) -> None:  # noqa: C901
        if self._state == self.STATE_DONE:
            self._publish_wheels(0.0, 0.0)
            return

        if not self._frame_is_fresh() or self.latest_frame is None:
            self.get_logger().warn("No fresh frame — stopping for safety.",
                                   throttle_duration_sec=2.0)
            self._publish_wheels(0.0, 0.0)
            return

        detections = self._detect_markers(self.latest_frame)

        # Build a dict of visible markers for quick lookup
        visible: dict[int, tuple] = {d[0]: d for d in detections}

        # -------------------------------------------------------------------
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
        # Skip markers we have already recorded.
        unseen = [mid for mid in WALL_DISTANCE_TO_CENTER
                  if mid not in self._seen_markers]

        # Check if any unseen marker is now visible
        for mid in unseen:
            if mid in visible:
                self._target_id = mid
                self.get_logger().info(
                    f"[SCANNING] Marker {mid} found → APPROACH"
                )
                self._publish_status(f"APPROACH:{mid}")
                self._state = self.STATE_APPROACH
                self._publish_wheels(0.0, 0.0)
                return

        # No unseen marker visible — if we have enough, go centre
        if len(self._seen_markers) >= MIN_MARKERS_FOR_CENTRE:
            self.get_logger().info(
                f"[SCANNING] Enough markers recorded "
                f"({len(self._seen_markers)}/{len(WALL_DISTANCE_TO_CENTER)}) → CENTRE"
            )
            self._begin_centre_phase()
            return

        # Slow CW pivot to search
        # Positive left, negative right → rotate clockwise (right turn)
        self._publish_wheels(SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED)

    # -- APPROACH ------------------------------------------------------------

    def _do_approach(self, visible: dict) -> None:
        if self._target_id not in visible:
            # Lost the marker briefly — slow CW search
            self.get_logger().warn(
                f"[APPROACH] Marker {self._target_id} lost — searching.",
                throttle_duration_sec=1.0,
            )
            self._publish_wheels(SCAN_PIVOT_SPEED * 0.5, -SCAN_PIVOT_SPEED * 0.5)
            return

        _, rvec, tvec = visible[self._target_id]
        x_offset = float(tvec[0])   # lateral: + = marker is to the RIGHT
        z_dist   = float(tvec[2])   # depth:   + = marker is in front

        error_z = z_dist - STANDOFF_M

        self.get_logger().info(
            f"[APPROACH] marker={self._target_id}  z={z_dist:.3f}m  "
            f"x={x_offset:.3f}m  err_z={error_z:.3f}m",
            throttle_duration_sec=0.5,
        )

        if abs(error_z) <= STANDOFF_TOL_M:
            # Arrived at standoff distance
            self._state = self.STATE_RECORD
            self._publish_wheels(0.0, 0.0)
            return

        # Proportional forward speed, with lateral correction mixed in.
        # Drive forward (positive z error means we're still too far away).
        # SIGN CONVENTION (matches _publish_wheels → Sabertooth):
        #   positive left/right → forward
        #   to correct rightward drift (x_offset > 0): slow right wheel, speed left
        #   i.e. steer = KP * x_offset;  left += steer;  right -= steer  ← turns right

        if error_z > 0:
            base = APPROACH_SPEED
        else:
            base = -APPROACH_SPEED  # overshot, back up (no lateral correction)
            x_offset = 0.0

        steer = KP_LATERAL * x_offset   # positive offset → steer right
        wl = base + steer
        wr = base - steer

        # Clamp
        max_mag = max(abs(wl), abs(wr))
        if max_mag > 100.0:
            scale = 100.0 / max_mag
            wl *= scale
            wr *= scale

        self._publish_wheels(wl, wr)

    # -- RECORD --------------------------------------------------------------

    def _do_record(self, visible: dict) -> None:
        if self._target_id not in visible:
            self.get_logger().warn(
                f"[RECORD] Marker {self._target_id} not visible at standoff — re-approach."
            )
            self._state = self.STATE_APPROACH
            return

        _, rvec, tvec = visible[self._target_id]
        z_dist = float(tvec[2])

        # The distance from the robot's CURRENT position to the enclosure centre
        # along the axis toward this marker is:
        #   dist_to_centre_via_this_wall = z_dist - (wall_to_centre - standoff)
        # But it is simpler to store the raw measured z at standoff; the
        # centre phase computes the actual distance to drive using:
        #   drive_dist = wall_to_centre - z_dist
        # (positive → centre is behind us relative to the wall)
        self._seen_markers[self._target_id] = z_dist
        self.get_logger().info(
            f"[RECORD] Marker {self._target_id} recorded at z={z_dist:.3f}m. "
            f"Total seen: {list(self._seen_markers.keys())}"
        )
        self._publish_status(f"RECORDED:{self._target_id}")

        # Check if we have seen all four markers
        if len(self._seen_markers) == len(WALL_DISTANCE_TO_CENTER):
            self.get_logger().info("[RECORD] All markers recorded → CENTRE")
            self._begin_centre_phase()
        else:
            self.get_logger().info("[RECORD] → SCANNING for next marker")
            self._state = self.STATE_SCANNING
            self._publish_wheels(0.0, 0.0)

    # -- CENTRE (dead-reckoning drive to enclosure centre) -------------------

    def _begin_centre_phase(self) -> None:
        """
        Compute the net displacement vector from the current robot position
        to the enclosure centre, then drive that distance forward.

        We use the NORTH marker (ID 0) as the primary reference if available.
        Otherwise we average all recorded markers.

        Strategy: face toward the centre (rotate) then drive straight.
        For simplicity, we drive toward the NORTH wall marker and stop at
        WALL_DISTANCE_TO_CENTER[0] from it, which puts us at the centre on
        the N-S axis.  If more markers are seen we refine the estimate.
        """
        if not self._seen_markers:
            self.get_logger().error("[CENTRE] No markers recorded — cannot centre.")
            return

        # Use the NORTH wall as the primary drive target if available,
        # otherwise pick the first recorded marker.
        if 0 in self._seen_markers:
            ref_id   = 0
        else:
            ref_id = next(iter(self._seen_markers))

        measured_z       = self._seen_markers[ref_id]       # m from robot to that wall
        desired_z        = WALL_DISTANCE_TO_CENTER[ref_id]  # m from centre to that wall
        self._centre_drive_dist_m = measured_z - desired_z  # how far to drive toward wall

        self.get_logger().info(
            f"[CENTRE] Using marker {ref_id}. "
            f"Drive {self._centre_drive_dist_m:.3f} m toward that wall."
        )

        # Turn to face the chosen reference marker before driving.
        # We do this by switching to ORIENT targeting that marker first,
        # then continuing to CENTRE_DRIVE.  For now we assume the robot is
        # roughly facing the right direction from the APPROACH phase.
        self._state = self.STATE_CENTRE
        self._centre_ref_id       = ref_id
        self._centre_drive_start_ns = self.get_clock().now().nanoseconds
        self._publish_status("CENTERING")

    def _do_centre(self) -> None:
        """Drive toward the reference wall marker until we have covered
        the required distance, using elapsed time × estimated speed."""
        if self._centre_drive_dist_m <= CENTRE_ARRIVAL_M:
            self.get_logger().info("[CENTRE] Already at centre → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            return

        elapsed_s = (self.get_clock().now().nanoseconds
                     - self._centre_drive_start_ns) / 1e9

        # Approximate velocity: CENTRE_SPEED command units × a generic
        # 0.005 m/s per unit (replace with dead_reckoning_cal.npz ratio
        # if you load it in this node — see gui_node for the load pattern).
        APPROX_MPS_PER_CMD = 0.005
        distance_driven    = elapsed_s * CENTRE_SPEED * APPROX_MPS_PER_CMD

        self.get_logger().info(
            f"[CENTRE] Driven {distance_driven:.3f} m of "
            f"{self._centre_drive_dist_m:.3f} m",
            throttle_duration_sec=0.5,
        )

        if distance_driven >= self._centre_drive_dist_m:
            self.get_logger().info("[CENTRE] Centre reached → ORIENT")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_ORIENT
            self._publish_status("ORIENT")
        else:
            self._publish_wheels(CENTRE_SPEED, CENTRE_SPEED)

    # -- ORIENT (align heading to face NORTH marker, ID 0) -------------------

    def _do_orient(self, visible: dict) -> None:
        """
        Spin in place until marker ID 0 (NORTH) is centred in the frame
        (x_offset ≈ 0).
        """
        if 0 not in visible:
            # Cannot see NORTH marker — slow CCW spin to search for it
            self.get_logger().info(
                "[ORIENT] NORTH marker not visible — spinning to find it.",
                throttle_duration_sec=1.0,
            )
            # CCW: left wheel backward, right wheel forward
            self._publish_wheels(-ORIENT_SPEED, ORIENT_SPEED)
            return

        _, rvec, tvec = visible[0]
        x_offset = float(tvec[0])   # positive = marker is to the RIGHT of camera

        self.get_logger().info(
            f"[ORIENT] NORTH marker x_offset={x_offset:.4f} m",
            throttle_duration_sec=0.5,
        )

        # Convert lateral offset to angular error estimate
        z_dist = max(float(tvec[2]), 0.1)
        angle_error = math.atan2(x_offset, z_dist)  # radians; + = marker right

        if abs(angle_error) <= ORIENT_TOL_RAD:
            self.get_logger().info("[ORIENT] Heading aligned → DONE")
            self._publish_wheels(0.0, 0.0)
            self._state = self.STATE_DONE
            self._publish_status("DONE")
            return

        # Turn to align:
        # marker is to the RIGHT (angle_error > 0) → turn CW → left fwd, right back
        # marker is to the LEFT  (angle_error < 0) → turn CCW → left back, right fwd
        if angle_error > 0:
            wl, wr =  ORIENT_SPEED, -ORIENT_SPEED   # CW
        else:
            wl, wr = -ORIENT_SPEED,  ORIENT_SPEED   # CCW

        self._publish_wheels(wl, wr)


# ---------------------------------------------------------------------------

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