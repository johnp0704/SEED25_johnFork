"""
commander_node.py  —  arbitrates wheel commands and drives Sabertooth hardware.

Tool integration
----------------
When the GTG controller reaches its target it publishes on /auger/activate.
The commander intercepts this, enters DRILLING mode, and:

  1. Immediately zeroes all wheel outputs (wheels stay at 0 for entire cycle).
  2. Forwards the activate signal to /tool/activate (tool_controller_node).
  3. Subscribes to /tool/status and waits for DONE or ERROR.
     • DONE  → resumes the mode that was active before drilling started, and
               publishes a 10-second cooldown to /gtg/cooldown so the GTG
               controller won't re-detect the same weed immediately.
     • ERROR → transitions to IDLE and waits for user input from the GUI.
               The GUI's /commander/ack subscription will show "IDLE".

Publishes /rehome/reset whenever the GUI switches INTO REHOME mode so the
aruco_rehoming_node restarts its scan from scratch.

Modes
-----
  IDLE        — all wheels stopped.
  REHOME      — aruco-based re-homing; uses rehome_cmd topic.
  OPTICAL     — optical path follower with GTG override when red detected.
  GTG         — pure go-to-goal; robot ONLY moves when the GTG controller is
                actively publishing commands (red weed detected).  No optical
                fallback — wheels stay at zero otherwise.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from demo_night.sabertooth import SaberToothMotorDriver

GTG_TIMEOUT_SEC         = 0.5
MODE_TIMEOUT_SEC        = 1.0
SABERTOOTH_CALL_RATE_HZ = 20.0

# How long (seconds) to suppress GTG detection after a successful drill.
GTG_COOLDOWN_SEC = 10.0


class CommanderNode(Node):

    # All valid modes the commander can be in.
    # DRILLING is an internal sub-mode — externally it looks like the prior
    # mode is paused, so the GUI still shows the last confirmed mode while
    # drilling.
    _VALID_GUI_MODES = {"IDLE", "REHOME", "OPTICAL", "GTG"}

    def __init__(self):
        super().__init__('commander_node')

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

        self.current_mode:   str = "IDLE"
        self.confirmed_mode: str = "IDLE"

        # Indicates the robot is mid-drill cycle — overrides all wheel commands
        # regardless of current_mode.
        self._drilling: bool = False

        # Mode to restore after a successful drill cycle.
        self._pre_drill_mode: str = "IDLE"

        self.cmd_gtg     = [0.0, 0.0]
        self.cmd_rehome  = [0.0, 0.0]
        self.cmd_optical = [0.0, 0.0]

        self.t_gtg     = 0
        self.t_rehome  = 0
        self.t_optical = 0

        self.gtg_active: bool = False

        # Publishers
        self.motor_pub    = self.create_publisher(
            Float32MultiArray, '/commander/wheel_cmd', 10)
        self.ack_pub      = self.create_publisher(String, '/commander/ack', 10)
        self.reset_pub    = self.create_publisher(String, '/rehome/reset', 10)
        self.tool_pub     = self.create_publisher(String, '/tool/activate', 10)
        self.cooldown_pub = self.create_publisher(String, '/gtg/cooldown', 10)

        # Subscriptions
        self.create_subscription(
            String, '/gui/system_state', self._gui_state_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/vision/gtg_cmd', self._gtg_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/vision/rehome_cmd', self._rehome_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/vision/optical_cmd', self._optical_cb, 10)

        # Auger trigger from GTG controller
        self.create_subscription(
            String, '/auger/activate', self._auger_activate_cb, 10)

        # Tool status from tool_controller_node
        self.create_subscription(
            String, '/tool/status', self._tool_status_cb, 10)

        self.create_timer(1.0 / SABERTOOTH_CALL_RATE_HZ, self._control_loop)
        self.get_logger().info(
            "Commander Node initialised.  Standing by in IDLE mode.")

    # ===================================================================
    # Callbacks — mode / wheel commands
    # ===================================================================

    def _gui_state_cb(self, msg: String) -> None:
        new_mode = msg.data.upper()
        if new_mode not in self._VALID_GUI_MODES:
            self.get_logger().warn(
                f"GUI requested unknown mode '{new_mode}' — ignored.")
            return

        # If the user manually commands IDLE while drilling, honour it:
        # abort any in-progress drill tracking and reset.
        if new_mode == "IDLE" and self._drilling:
            self.get_logger().warn(
                "GUI commanded IDLE during active drill — "
                "aborting drill tracking and stopping.")
            self._drilling = False

        previous_mode     = self.current_mode
        self.current_mode = new_mode
        self.get_logger().info(f"Mode change requested: {new_mode}")

        if new_mode == "REHOME" and previous_mode != "REHOME":
            reset_msg = String(); reset_msg.data = "reset"
            self.reset_pub.publish(reset_msg)
            self.get_logger().info(
                "Published /rehome/reset — rehome scan restarting.")

        ack = String(); ack.data = new_mode
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

    # ===================================================================
    # Callbacks — tool pipeline
    # ===================================================================

    def _auger_activate_cb(self, _msg: String) -> None:
        """
        GTG controller signals it has reached the weed target.

        Accepts the trigger if:
          • A drill cycle is not already running, AND
          • The robot is in OPTICAL or GTG mode with GTG actively in control
            (a fresh GTG wheel command arrived within GTG_TIMEOUT_SEC).
        """
        if self._drilling:
            self.get_logger().warn(
                "[TOOL] /auger/activate received but drill already in progress — "
                "ignoring duplicate.")
            return

        gtg_is_in_control = (
            self.current_mode in ("OPTICAL", "GTG") and self.gtg_active
        )
        if not gtg_is_in_control:
            self.get_logger().warn(
                f"[TOOL] /auger/activate received but GTG is not in control "
                f"(mode='{self.current_mode}', gtg_active={self.gtg_active}) "
                "— ignoring.")
            return

        self.get_logger().info(
            "[TOOL] Weed reached — entering DRILLING mode.  "
            "Wheels zeroed, forwarding activate to tool_controller_node.")

        self._pre_drill_mode = self.current_mode
        self._drilling = True

        activate_msg = String()
        activate_msg.data = "activate"
        self.tool_pub.publish(activate_msg)

    def _tool_status_cb(self, msg: String) -> None:
        """
        Receives status updates from tool_controller_node.

        Only the terminal states (DONE, ERROR) require action here.
        Intermediate states (HOMING, DRILLING) are just logged.
        """
        status = msg.data.upper()

        if status in ("HOMING", "DRILLING"):
            self.get_logger().info(f"[TOOL] Tool status: {status}")
            return

        if not self._drilling:
            return

        if status == "DONE":
            resume_mode = self._pre_drill_mode if self._pre_drill_mode else "OPTICAL"
            self.get_logger().info(
                f"[TOOL] Drill cycle DONE.  "
                f"Resuming {resume_mode} mode with {GTG_COOLDOWN_SEC}s GTG cooldown.")
            self._drilling = False

            self.current_mode   = resume_mode
            self.confirmed_mode = resume_mode
            ack = String(); ack.data = resume_mode
            self.ack_pub.publish(ack)

            cooldown_msg = String()
            cooldown_msg.data = str(GTG_COOLDOWN_SEC)
            self.cooldown_pub.publish(cooldown_msg)
            self.get_logger().info(
                f"[TOOL] Published GTG cooldown: {GTG_COOLDOWN_SEC}s.")

        elif status == "ERROR":
            self.get_logger().error(
                "[TOOL] Drill cycle ERROR (RC_MOVE_UNSAFE or serial failure).  "
                "Transitioning to IDLE — awaiting user input.")
            self._drilling = False

            self.current_mode   = "IDLE"
            self.confirmed_mode = "IDLE"
            ack = String(); ack.data = "IDLE"
            self.ack_pub.publish(ack)

        else:
            self.get_logger().warn(f"[TOOL] Unrecognised tool status: '{status}'")

    # ===================================================================
    # Control loop
    # ===================================================================

    def _control_loop(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        gtg_age_sec     = (now_ns - self.t_gtg) / 1e9
        self.gtg_active = (gtg_age_sec < GTG_TIMEOUT_SEC)

        # ---------------------------------------------------------------
        # DRILLING overrides everything — wheels stay at zero.
        # ---------------------------------------------------------------
        if self._drilling:
            self._apply_wheels(0.0, 0.0)
            return

        # ---------------------------------------------------------------
        # Normal mode arbitration
        # ---------------------------------------------------------------
        final_cmd = [0.0, 0.0]

        if self.current_mode == "IDLE":
            final_cmd = [0.0, 0.0]

        elif self.current_mode == "REHOME":
            final_cmd = self._safe_cmd(
                self.cmd_rehome, self.t_rehome, now_ns, "REHOME")

        elif self.current_mode == "OPTICAL":
            # GTG overrides optical when red is actively detected.
            if self.gtg_active:
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG override active (OPTICAL mode).",
                    throttle_duration_sec=1.0)
            else:
                final_cmd = self._safe_cmd(
                    self.cmd_optical, self.t_optical, now_ns, "OPTICAL")

        elif self.current_mode == "GTG":
            # Pure go-to-goal: ONLY move when GTG is actively publishing.
            # No optical fallback — wheels stay at zero if no red detected.
            if self.gtg_active:
                final_cmd = self.cmd_gtg
                self.get_logger().info(
                    "GTG active — driving toward weed.",
                    throttle_duration_sec=1.0)
            else:
                # No red detected — hold position silently.
                final_cmd = [0.0, 0.0]

        self._apply_wheels(float(final_cmd[0]), float(final_cmd[1]))

    # ===================================================================
    # Helpers
    # ===================================================================

    def _apply_wheels(self, wl: float, wr: float) -> None:
        if self._motors is not None:
            self._motors.updateMotorSpeed(wl, wr)
        out = Float32MultiArray()
        out.data = [wl, wr]
        self.motor_pub.publish(out)

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
        if self._motors is not None:
            try:
                self._motors.all_motors_off()
            except Exception:
                pass
        super().destroy_node()


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