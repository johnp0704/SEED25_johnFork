"""
tool_controller_node.py

Bridges the ROS2 tool pipeline to the ESP32 auger/drill controller over
a USB serial link.

Startup homing
--------------
The stepper is homed ONCE when the node starts.  Subsequent drill cycles
skip the home step entirely — the ESP32 maintains position between cycles.
This eliminates the ~10-45s homing delay on every weed detection.

The GUI exposes two manual overrides via separate topics:
  /tool/rehome — triggers an on-demand home (e.g. after an E-stop recovery
                 or if the head is suspected to be in an unknown position).
  /tool/estop  — immediately sends "estop\n" to the ESP32 and kills any
                 in-progress drill sequence.

ESP32 serial protocol (115 200 baud, newline-terminated):
  Send  "home\n"  → ESP32 homes + moves to calibrated offset → responds "0\n"
  Send  "drill\n" → ESP32 runs full drill cycle (feed + retract)
                    → responds "0\n" (RC_OK) or "1\n" (RC_MOVE_UNSAFE)
  Send  "estop\n" → ESP32 kills motor immediately
  Either side can receive "ESTOP\n" if the ESP32 triggers an emergency stop.

Drill sequence on every /tool/activate message:
  1. Flush stale serial data
  2. Send "drill\n", wait for "0"/"1" → publish DONE or ERROR
  3. Return to IDLE

Topics
------
Subscribes : /tool/activate  (std_msgs/String)  — any message triggers a cycle
             /tool/rehome    (std_msgs/String)  — triggers on-demand home
             /tool/estop     (std_msgs/String)  — immediate motor kill
Publishes  : /tool/status    (std_msgs/String)  — IDLE | HOMING | DRILLING | DONE | ERROR

Parameters
----------
serial_port   str   /dev/ttyUSB0
baud_rate     int   115200
"""
from __future__ import annotations
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

# ============================================================
# Timing budgets
# ============================================================
HOME_TIMEOUT_SEC  = 45.0
DRILL_TIMEOUT_SEC = 90.0


class ToolControllerNode(Node):

    _STATE_IDLE = "IDLE"
    _STATE_BUSY = "BUSY"

    def __init__(self):
        super().__init__('tool_controller_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',   115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self._ser: serial.Serial | None = None
        self._serial_ok = False
        self._serial_lock = threading.Lock()   # guards all serial I/O

        if _SERIAL_AVAILABLE:
            try:
                self._ser = serial.Serial(
                    port=port,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1.0,
                )
                self._serial_ok = True
                self.get_logger().info(
                    f"ESP32 serial opened: {port} @ {baud} baud.")
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to open serial port {port}: {exc}")
        else:
            self.get_logger().error(
                "pyserial not installed — cannot control tool.  "
                "Install with: pip install pyserial --break-system-packages")

        self._state      = self._STATE_BUSY   # busy until startup home finishes
        self._state_lock = threading.Lock()

        self._status_pub = self.create_publisher(String, '/tool/status', 10)

        self.create_subscription(
            String, '/tool/activate', self._activate_cb, 10)
        self.create_subscription(
            String, '/tool/rehome',   self._rehome_cb,   10)
        self.create_subscription(
            String, '/tool/estop',    self._estop_cb,    10)

        self._publish_status("HOMING")
        self.get_logger().info(
            "Tool Controller Node ready — running startup home (one-time).")

        threading.Thread(target=self._run_startup_home, daemon=True).start()

    # -----------------------------------------------------------------------
    # Startup homing — runs ONCE
    # -----------------------------------------------------------------------

    def _run_startup_home(self) -> None:
        """
        Runs once at node startup.  After this completes successfully, the
        stepper position is known and no further homing is needed before
        each drill cycle.
        """
        try:
            if not self._serial_ok or self._ser is None:
                self.get_logger().error(
                    "[TOOL] No serial — cannot run startup home.")
                self._publish_status("ERROR")
                return

            time.sleep(2.0)   # let ESP32 finish booting

            with self._serial_lock:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self.get_logger().info("[TOOL] Startup: sending HOME.")
                self._ser.write(b"home\n")
                resp = self._read_response_locked(HOME_TIMEOUT_SEC)

            self.get_logger().info(f"[TOOL] Startup home response: '{resp}'")

            if resp == "0":
                self.get_logger().info(
                    "[TOOL] Startup homing complete — ready.")
                self._publish_status("IDLE")
            else:
                self.get_logger().error(
                    f"[TOOL] Startup homing failed ('{resp}') — ERROR.")
                self._publish_status("ERROR")

        except Exception as exc:
            self.get_logger().error(f"[TOOL] Startup home exception: {exc}")
            self._publish_status("ERROR")

        finally:
            with self._state_lock:
                self._state = self._STATE_IDLE

    # -----------------------------------------------------------------------
    # On-demand rehome (GUI button)
    # -----------------------------------------------------------------------

    def _rehome_cb(self, _msg: String) -> None:
        """
        Triggered by the GUI REHOME DRILL button.
        Only runs if the tool is currently idle.
        """
        with self._state_lock:
            if self._state != self._STATE_IDLE:
                self.get_logger().warn(
                    "[TOOL] Rehome requested but tool is busy — ignoring.")
                return
            self._state = self._STATE_BUSY

        threading.Thread(target=self._run_on_demand_home, daemon=True).start()

    def _run_on_demand_home(self) -> None:
        try:
            if not self._serial_ok or self._ser is None:
                self._publish_status("ERROR")
                return

            self.get_logger().info("[TOOL] On-demand HOME requested by GUI.")
            self._publish_status("HOMING")

            with self._serial_lock:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self._ser.write(b"home\n")
                resp = self._read_response_locked(HOME_TIMEOUT_SEC)

            self.get_logger().info(f"[TOOL] On-demand home response: '{resp}'")

            if resp == "0":
                self.get_logger().info("[TOOL] On-demand home complete.")
                self._publish_status("IDLE")
            else:
                self.get_logger().error(
                    f"[TOOL] On-demand home failed ('{resp}').")
                self._publish_status("ERROR")

        except Exception as exc:
            self.get_logger().error(f"[TOOL] On-demand home exception: {exc}")
            self._publish_status("ERROR")

        finally:
            with self._state_lock:
                self._state = self._STATE_IDLE

    # -----------------------------------------------------------------------
    # E-stop (GUI button) — interrupts any in-progress sequence
    # -----------------------------------------------------------------------

    def _estop_cb(self, _msg: String) -> None:
        """
        Sends "estop\\n" immediately on the serial line regardless of current
        state.  Uses the serial lock to avoid colliding with an in-progress
        readline, but does NOT wait for a response — this must be fast.
        """
        self.get_logger().error("[TOOL] E-STOP commanded by GUI!")
        self._publish_status("ERROR")

        if not self._serial_ok or self._ser is None:
            return

        try:
            with self._serial_lock:
                self._ser.reset_output_buffer()
                self._ser.write(b"estop\n")
        except Exception as exc:
            self.get_logger().error(f"[TOOL] E-stop serial write failed: {exc}")

        # Force state back to IDLE so the system can recover after the operator
        # re-homes and resumes.
        with self._state_lock:
            self._state = self._STATE_IDLE

    # -----------------------------------------------------------------------
    # Activate callback — drill cycle (NO homing before drill)
    # -----------------------------------------------------------------------

    def _activate_cb(self, _msg: String) -> None:
        with self._state_lock:
            if self._state != self._STATE_IDLE:
                self.get_logger().warn(
                    "[TOOL] Activate received but tool is busy — ignoring.")
                return
            self._state = self._STATE_BUSY

        threading.Thread(target=self._run_drill_sequence, daemon=True).start()

    def _run_drill_sequence(self) -> None:
        """
        Drill-only sequence — no homing step.

        The stepper was homed at startup (or by an explicit on-demand home).
        Skipping the home here saves 10-45 seconds per weed detection.
        """
        try:
            if not self._serial_ok or self._ser is None:
                self.get_logger().error("[TOOL] No serial — publishing ERROR.")
                self._publish_status("ERROR")
                return

            self.get_logger().info("[TOOL] Sending DRILL command (no pre-home).")
            self._publish_status("DRILLING")

            with self._serial_lock:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self._ser.write(b"drill\n")
                drill_resp = self._read_response_locked(DRILL_TIMEOUT_SEC)

            self.get_logger().info(f"[TOOL] Drill response: '{drill_resp}'")

            if drill_resp == "0":
                self.get_logger().info("[TOOL] Drill cycle complete — DONE.")
                self._publish_status("DONE")
            else:
                self.get_logger().error(
                    f"[TOOL] Drill failed ('{drill_resp}') — ERROR.")
                self._publish_status("ERROR")

        except Exception as exc:
            self.get_logger().error(f"[TOOL] Drill exception: {exc}")
            self._publish_status("ERROR")

        finally:
            with self._state_lock:
                self._state = self._STATE_IDLE

    # -----------------------------------------------------------------------
    # Serial helpers
    # -----------------------------------------------------------------------

    def _read_response_locked(self, timeout_s: float) -> str:
        """
        Read lines until a non-empty response or deadline.
        MUST be called with self._serial_lock already held.
        """
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            self._ser.timeout = min(remaining, 1.0)

            try:
                raw  = self._ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
            except serial.SerialException as exc:
                self.get_logger().error(f"[TOOL] Serial read error: {exc}")
                return "SERIAL_ERROR"

            if not line:
                continue

            self.get_logger().info(f"[TOOL] Serial ← '{line}'")

            if line == "ESTOP":
                self.get_logger().error("[TOOL] ESTOP from ESP32!")
                return "ESTOP"

            return line

        return "TIMEOUT"

    # -----------------------------------------------------------------------

    def _publish_status(self, status: str) -> None:
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[TOOL] Status → {status}")

    def destroy_node(self) -> None:
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToolControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()