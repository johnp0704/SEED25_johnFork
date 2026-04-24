"""
tool_controller_node.py

Bridges the ROS2 tool pipeline to the ESP32 auger/drill controller over
a USB serial link.

ESP32 serial protocol (115 200 baud, newline-terminated):
  Send  "home\\n"  → ESP32 homes + moves to calibrated offset
                    → responds "0\\n"  (RC_OK) when done
  Send  "drill\\n" → ESP32 runs full drill cycle (feed + retract)
                    → responds "0\\n"  (RC_OK) or "1\\n"  (RC_MOVE_UNSAFE)
  Either side can receive "ESTOP\\n" if the ESP32 triggers an emergency stop.

Drill sequence on every /tool/activate message:
  1. Flush stale serial data
  2. Send "home\\n", wait for "0"      → publish HOMING, then DRILLING
  3. Send "drill\\n", wait for "0"/"1" → publish DONE or ERROR
  4. Return to IDLE

If the serial port cannot be opened the node starts in a degraded mode:
it still publishes "ERROR" for every activate request so the rest of the
pipeline (particularly the GTG cooldown) can proceed normally rather than
hanging forever in DRILLING_WAIT.

Topics
------
Subscribes : /tool/activate  (std_msgs/String)  — any message triggers a cycle
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
# Timing budgets (generous — homing can take 10+ seconds)
# ============================================================
HOME_TIMEOUT_SEC  = 45.0
DRILL_TIMEOUT_SEC = 90.0


class ToolControllerNode(Node):

    _STATE_IDLE  = "IDLE"
    _STATE_BUSY  = "BUSY"

    def __init__(self):
        super().__init__('tool_controller_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',   115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self._ser: serial.Serial | None = None
        self._serial_ok = False

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
                    f"Failed to open serial port {port}: {exc}\n"
                    "Node will publish ERROR for every activate request.")
        else:
            self.get_logger().error(
                "pyserial not installed — cannot control tool.  "
                "Install with: pip install pyserial --break-system-packages")

        self._state      = self._STATE_IDLE
        self._state_lock = threading.Lock()

        self._status_pub = self.create_publisher(String, '/tool/status',    10)
        self.create_subscription(String, '/tool/activate', self._activate_cb, 10)

        # Publish initial IDLE so subscribers know we're alive
        self._publish_status("IDLE")
        self.get_logger().info("Tool Controller Node ready.")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _activate_cb(self, _msg: String) -> None:
        """Received on /tool/activate.  Starts the drill sequence if idle."""
        with self._state_lock:
            if self._state != self._STATE_IDLE:
                self.get_logger().warn(
                    "[TOOL] Activate request received but tool is already busy — ignoring.")
                return
            self._state = self._STATE_BUSY

        t = threading.Thread(target=self._run_drill_sequence, daemon=True)
        t.start()

    # -----------------------------------------------------------------------
    # Background drill-sequence thread
    # -----------------------------------------------------------------------

    def _run_drill_sequence(self) -> None:
        """
        Runs entirely in a daemon thread so blocking serial I/O doesn't
        stall the ROS2 event loop.

        Flow:
          1. home  → wait for "0"
          2. drill → wait for "0" or "1"
          3. Publish DONE / ERROR and return to IDLE
        """
        try:
            if not self._serial_ok or self._ser is None:
                self.get_logger().error("[TOOL] No serial connection — publishing ERROR.")
                self._publish_status("ERROR")
                return

            # Flush any stale bytes from a previous run
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            # ------- HOMING -------
            self.get_logger().info("[TOOL] Sending HOME command.")
            self._publish_status("HOMING")

            self._ser.write(b"home\n")
            home_resp = self._read_response(HOME_TIMEOUT_SEC)
            self.get_logger().info(f"[TOOL] Home response: '{home_resp}'")

            if home_resp != "0":
                self.get_logger().error(
                    f"[TOOL] Homing failed (response='{home_resp}') — aborting.")
                self._publish_status("ERROR")
                return

            # ------- DRILLING -------
            self.get_logger().info("[TOOL] Sending DRILL command.")
            self._publish_status("DRILLING")

            self._ser.write(b"drill\n")
            drill_resp = self._read_response(DRILL_TIMEOUT_SEC)
            self.get_logger().info(f"[TOOL] Drill response: '{drill_resp}'")

            if drill_resp == "0":
                self.get_logger().info("[TOOL] Drill cycle complete — DONE.")
                self._publish_status("DONE")
            else:
                self.get_logger().error(
                    f"[TOOL] Drill cycle failed (response='{drill_resp}') — ERROR.")
                self._publish_status("ERROR")

        except Exception as exc:
            self.get_logger().error(f"[TOOL] Unexpected exception: {exc}")
            self._publish_status("ERROR")

        finally:
            with self._state_lock:
                self._state = self._STATE_IDLE

    # -----------------------------------------------------------------------
    # Serial helpers
    # -----------------------------------------------------------------------

    def _read_response(self, timeout_s: float) -> str:
        """
        Reads lines from serial until a non-empty response arrives or the
        deadline is exceeded.

        Returns the stripped response string, or "TIMEOUT" / "ESTOP" as
        appropriate.
        """
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            # Keep individual read timeouts short so we don't miss the deadline
            self._ser.timeout = min(remaining, 1.0)

            try:
                raw  = self._ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
            except serial.SerialException as exc:
                self.get_logger().error(f"[TOOL] Serial read error: {exc}")
                return "SERIAL_ERROR"

            if not line:
                continue   # empty line / timeout on this 1-second slice — keep waiting

            self.get_logger().info(f"[TOOL] Serial ← '{line}'")

            if line == "ESTOP":
                self.get_logger().error("[TOOL] ESTOP received from ESP32!")
                return "ESTOP"

            return line   # "0", "1", or any other response

        return "TIMEOUT"

    # -----------------------------------------------------------------------
    # Publisher helper
    # -----------------------------------------------------------------------

    def _publish_status(self, status: str) -> None:
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[TOOL] Status → {status}")

    # -----------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


# ===========================================================================

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