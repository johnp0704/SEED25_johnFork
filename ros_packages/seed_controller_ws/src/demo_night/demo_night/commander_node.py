"""
commander_node.py  —  arbitrates wheel commands and drives Sabertooth hardware.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

# Sabertooth lives in the demo_night package alongside this file
from demo_night.sabertooth import SaberToothMotorDriver


# ===========================================================================
# Tuning constants
# ===========================================================================

GTG_TIMEOUT_SEC  = 0.5
MODE_TIMEOUT_SEC = 1.0
GTG_OVERRIDABLE_MODES = frozenset({"OPTICAL", "TRAJECTORY"})

# Call rate must match the commander's 20 Hz timer
SABERTOOTH_CALL_RATE_HZ = 20.0


class CommanderNode(Node):

    def __init__(self):
        super().__init__('commander_node')

        # --- Sabertooth motor driver ---------------------------------------
        # motor1_reversed=True, motor2_reversed=True matches the teleop node
        # and the physical wiring confirmed by motor_test_node.
        try:
            self._motors = SaberToothMotorDriver(
                motor1_reversed=True,
                motor2_reversed=True,
                accel_rate=80.0,
                decel_rate=120.0,
                call_rate_hz=SABERTOOTH_CALL_RATE_HZ,
            )
            self.get_logger().info("Sabertooth initialised on /dev/ttyTHS0.")
        except Exception as exc:
            self.get_logger().error(f"Sabertooth FAILED to initialise: {exc}")
            self._motors = None

        # --- State --------------------------------------------------------
        self.current_mode:   str = "IDLE"
        self.confirmed_mode: str = "IDLE"

        self.cmd_gtg        = [0.0, 0.0]
        self.cmd_rehome     = [0.0, 0.0]
        self.cmd_optical    = [0.0, 0.0]
        self.cmd_trajectory = [0.0, 0.0]

        self.t_gtg        = 0
        self.t_rehome     = 0
        self.t_optical    = 0
        self.t_trajectory = 0

        self.gtg_active: bool = False

        # --- Publishers ---------------------------------------------------
        # Still publish wheel_cmd so the GUI virtual twin keeps working
        self.motor_pub = self.create_publisher(
            Float32MultiArray, '/commander/wheel_cmd', 10)
        self.ack_pub = self.create_publisher(String, '/commander/ack', 10)

        # --- Subscriptions ------------------------------------------------
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

        # --- Control loop at 20 Hz ----------------------------------------
        self.create_timer(1.0 / SABERTOOTH_CALL_RATE_HZ, self._control_loop)

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

        gtg_age_sec     = (now_ns - self.t_gtg) / 1e9
        self.gtg_active = (gtg_age_sec < GTG_TIMEOUT_SEC)

        final_cmd = [0.0, 0.0]

        if self.current_mode == "IDLE":
            final_cmd = [0.0, 0.0]

        elif self.current_mode == "REHOME":
            final_cmd = self._safe_cmd(
                self.cmd_rehome, self.t_rehome, now_ns, "REHOME")

        elif self.current_mode == "OPTICAL":
            if self.gtg_active:
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG override active (OPTICAL mode).",
                    throttle_duration_sec=1.0)
            else:
                final_cmd = self._safe_cmd(
                    self.cmd_optical, self.t_optical, now_ns, "OPTICAL")

        elif self.current_mode == "TRAJECTORY":
            if self.gtg_active:
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG override active (TRAJECTORY mode).",
                    throttle_duration_sec=1.0)
            else:
                final_cmd = self._safe_cmd(
                    self.cmd_trajectory, self.t_trajectory, now_ns, "TRAJECTORY")

        # --- Drive hardware -----------------------------------------------
        wl = float(final_cmd[0])
        wr = float(final_cmd[1])

        if self._motors is not None:
            self._motors.updateMotorSpeed(wl, wr)

        # --- Publish for virtual twin -------------------------------------
        out = Float32MultiArray()
        out.data = [wl, wr]
        self.motor_pub.publish(out)

    # ===================================================================
    # Helpers
    # ===================================================================

    def _safe_cmd(
        self, cmd: list, last_time_ns: int, now_ns: int, label: str
    ) -> list:
        age_sec = (now_ns - last_time_ns) / 1e9
        if last_time_ns == 0 or age_sec > MODE_TIMEOUT_SEC:
            self.get_logger().warn(
                f"{label} controller timed out ({age_sec:.2f}s) — stopping.",
                throttle_duration_sec=2.0,
            )
            return [0.0, 0.0]
        return cmd

    def destroy_node(self) -> None:
        """Emergency stop on shutdown."""
        if self._motors is not None:
            try:
                self._motors.all_motors_off()
            except Exception:
                pass
        super().destroy_node()


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