"""
commander_node.py

The single point of authority for all wheel commands.  Every sub-controller
publishes on its own topic; this node arbitrates priority and forwards exactly
one command to the motor driver at 20 Hz.

Priority order (highest → lowest)
----------------------------------
1. GTG (go-to-goal) — overrides OPTICAL and TRAJECTORY only.
   It does NOT override REHOME or IDLE.
2. Active mode command (REHOME / OPTICAL / TRAJECTORY).
3. IDLE → zero command.

Safety
------
Each mode has its own last-heard timestamp.  If the active controller stops
publishing for > MODE_TIMEOUT_SEC, the commander halts the robot and logs a
warning, rather than replaying the last stale command.

Acknowledgement
---------------
When the GUI publishes a new mode on /gui/system_state, the commander echoes
the accepted mode on /commander/ack so the GUI can display confirmed state
rather than just the requested state.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


# ===========================================================================
# Tuning constants
# ===========================================================================

# GTG detection is considered active if a command arrived within this window.
GTG_TIMEOUT_SEC  = 0.5   # 500 ms

# Any active sub-controller must publish within this window or we E-stop.
MODE_TIMEOUT_SEC = 1.0   # 1 s

# Modes where GTG is allowed to take override control.
GTG_OVERRIDABLE_MODES = frozenset({"OPTICAL", "TRAJECTORY"})

# ===========================================================================


class CommanderNode(Node):

    def __init__(self):
        super().__init__('commander_node')

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------
        self.current_mode: str  = "IDLE"   # as set by the GUI
        self.confirmed_mode: str = "IDLE"  # echoed back after acceptance

        # Per-mode command buffers
        self.cmd_gtg        = [0.0, 0.0]
        self.cmd_rehome     = [0.0, 0.0]
        self.cmd_optical    = [0.0, 0.0]
        self.cmd_trajectory = [0.0, 0.0]

        # Per-mode timestamps (nanoseconds, initialised to 0 = never received)
        self.t_gtg        = 0
        self.t_rehome     = 0
        self.t_optical    = 0
        self.t_trajectory = 0

        # GTG active flag — True only while within the timeout window
        self.gtg_active: bool = False

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        # The ONLY node that drives the physical motors
        self.motor_pub = self.create_publisher(
            Float32MultiArray, '/commander/wheel_cmd', 10)

        # Acknowledgement back to GUI
        self.ack_pub = self.create_publisher(String, '/commander/ack', 10)

        # ---------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------
        self.create_subscription(
            String, '/gui/system_state', self._gui_state_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/vision/gtg_cmd', self._gtg_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/vision/rehome_cmd', self._rehome_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/vision/optical_cmd', self._optical_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/nav/trajectory_cmd', self._trajectory_cb, 10)

        # ---------------------------------------------------------------
        # Control loop — 20 Hz
        # ---------------------------------------------------------------
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            "Commander Node initialised.  Standing by in IDLE mode.")

    # ===================================================================
    # Callbacks
    # ===================================================================

    def _gui_state_cb(self, msg: String) -> None:
        new_mode = msg.data.upper()
        valid = {"IDLE", "REHOME", "OPTICAL", "TRAJECTORY"}

        if new_mode not in valid:
            self.get_logger().warn(
                f"GUI requested unknown mode '{new_mode}' — ignored.")
            return

        self.current_mode = new_mode
        self.get_logger().info(f"Mode change requested: {new_mode}")

        # Echo acknowledgement back to GUI immediately
        ack = String()
        ack.data = new_mode
        self.ack_pub.publish(ack)
        self.confirmed_mode = new_mode

    def _gtg_cb(self, msg: Float32MultiArray) -> None:
        self.cmd_gtg = [msg.data[0], msg.data[1]]
        self.t_gtg   = self.get_clock().now().nanoseconds

    def _rehome_cb(self, msg: Float32MultiArray) -> None:
        self.cmd_rehome = [msg.data[0], msg.data[1]]
        self.t_rehome   = self.get_clock().now().nanoseconds

    def _optical_cb(self, msg: Float32MultiArray) -> None:
        self.cmd_optical = [msg.data[0], msg.data[1]]
        self.t_optical   = self.get_clock().now().nanoseconds

    def _trajectory_cb(self, msg: Float32MultiArray) -> None:
        self.cmd_trajectory = [msg.data[0], msg.data[1]]
        self.t_trajectory   = self.get_clock().now().nanoseconds

    # ===================================================================
    # Control loop
    # ===================================================================

    def _control_loop(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # ------------------------------------------------------------------
        # 1. Update GTG active flag from its dedicated timeout
        # ------------------------------------------------------------------
        gtg_age_sec = (now_ns - self.t_gtg) / 1e9
        self.gtg_active = (gtg_age_sec < GTG_TIMEOUT_SEC)

        # ------------------------------------------------------------------
        # 2. Determine final command
        # ------------------------------------------------------------------
        final_cmd = [0.0, 0.0]

        if self.current_mode == "IDLE":
            # Always zero in IDLE — GTG does NOT override idle.
            final_cmd = [0.0, 0.0]

        elif self.current_mode == "REHOME":
            # GTG does NOT override rehoming (don't abandon a safety sequence).
            final_cmd = self._safe_cmd(
                self.cmd_rehome, self.t_rehome, now_ns, "REHOME")

        elif self.current_mode == "OPTICAL":
            if self.gtg_active:
                # GTG overrides optical path following
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG override active (OPTICAL mode).",
                    throttle_duration_sec=1.0)
            else:
                final_cmd = self._safe_cmd(
                    self.cmd_optical, self.t_optical, now_ns, "OPTICAL")

        elif self.current_mode == "TRAJECTORY":
            if self.gtg_active:
                # GTG overrides trajectory following
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG override active (TRAJECTORY mode).",
                    throttle_duration_sec=1.0)
            else:
                final_cmd = self._safe_cmd(
                    self.cmd_trajectory, self.t_trajectory, now_ns, "TRAJECTORY")

        # ------------------------------------------------------------------
        # 3. Publish to motor driver
        # ------------------------------------------------------------------
        out = Float32MultiArray()
        out.data = [float(final_cmd[0]), float(final_cmd[1])]
        self.motor_pub.publish(out)

    # ===================================================================
    # Helpers
    # ===================================================================

    def _safe_cmd(
        self,
        cmd: list,
        last_time_ns: int,
        now_ns: int,
        label: str,
    ) -> list:
        """
        Return cmd if it is fresh, otherwise return a zero stop command.
        Each mode has its own timestamp so a live-but-wrong controller
        cannot mask a crashed controller.
        """
        age_sec = (now_ns - last_time_ns) / 1e9
        if last_time_ns == 0 or age_sec > MODE_TIMEOUT_SEC:
            self.get_logger().warn(
                f"{label} controller timed out ({age_sec:.2f}s) — stopping.",
                throttle_duration_sec=2.0,
            )
            return [0.0, 0.0]
        return cmd


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()