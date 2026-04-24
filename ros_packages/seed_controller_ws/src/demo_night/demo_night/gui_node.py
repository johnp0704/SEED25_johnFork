"""
gui_node.py

Commander GUI for the autonomous weeding robot.

Changes vs. previous version
-----------------------------
* Two new tool-control buttons:
    - "🔁 REHOME DRILL"  — publishes to /tool/rehome  (on-demand stepper home)
    - "🛑 E-STOP DRILL"  — publishes to /tool/estop   (immediate motor kill)
  These are wired to the matching subscribers in tool_controller_node.

* TELEOP mode added:
    - New "🕹 TELEOP" mode button in the commander panel.
    - On-screen D-pad (↑ ↓ ← →) that publishes /vision/teleop_cmd while
      held, then publishes [0,0] on release.
    - commander_node arbitrates teleop_cmd in its control loop.
    - No terminal raw-mode required — buttons live in the Qt window.

Modes
-----
  IDLE        — all wheels stopped.
  REHOME      — aruco-based re-homing sequence.
  OPTICAL     — optical path follower; GTG overrides when red detected.
  GTG         — pure go-to-goal; robot only moves when red weed is detected.
  TELEOP      — GUI D-pad drives robot directly.

Topics
------
Publishes  : /gui/system_state        (std_msgs/String)
             /tool/rehome             (std_msgs/String)
             /tool/estop              (std_msgs/String)
             /vision/teleop_cmd       (std_msgs/Float32MultiArray)
Subscribes : /commander/wheel_cmd     (std_msgs/Float32MultiArray)
             /commander/ack           (std_msgs/String)
             /rehome/status           (std_msgs/String)
             /tool/status             (std_msgs/String)
"""
from __future__ import annotations
import sys
import math
import os
import time
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton,
    QVBoxLayout, QHBoxLayout, QWidget, QLabel,
    QGridLayout, QGroupBox, QFrame,
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QPainter, QColor, QPen, QFont


# ===========================================================================
# Default dead-reckoning constants (used only if the .npz file is missing)
# ===========================================================================
DEFAULT_CMD_TO_MPS = 0.005
DEFAULT_WHEEL_BASE = 0.40

TELEOP_DRIVE_SPEED = 50.0
TELEOP_TURN_SPEED  = 50.0


# ===========================================================================
# Qt signal bridge — ROS2 callbacks → Qt main thread
# ===========================================================================

class _Signals(QObject):
    wheels_updated  = pyqtSignal(float, float)
    mode_confirmed  = pyqtSignal(str)
    rehome_status   = pyqtSignal(str)
    tool_status     = pyqtSignal(str)


# ===========================================================================
# ROS2 worker node (runs in a background thread)
# ===========================================================================

class ROS2Node(Node):

    def __init__(self, signals: _Signals):
        super().__init__('pyqt_gui_node')
        self._signals = signals

        self.state_pub    = self.create_publisher(String, '/gui/system_state', 10)
        self.rehome_pub   = self.create_publisher(String, '/tool/rehome',       10)
        self.estop_pub    = self.create_publisher(String, '/tool/estop',        10)
        self.teleop_pub   = self.create_publisher(
            Float32MultiArray, '/vision/teleop_cmd', 10)

        self.create_subscription(
            Float32MultiArray, '/commander/wheel_cmd', self._wheel_cb, 10)
        self.create_subscription(
            String, '/commander/ack',    self._ack_cb,    10)
        self.create_subscription(
            String, '/rehome/status',    self._rehome_cb, 10)
        self.create_subscription(
            String, '/tool/status',      self._tool_cb,   10)

    def request_mode(self, mode: str) -> None:
        msg = String(); msg.data = mode
        self.state_pub.publish(msg)

    def send_rehome_drill(self) -> None:
        msg = String(); msg.data = "rehome"
        self.rehome_pub.publish(msg)

    def send_estop_drill(self) -> None:
        msg = String(); msg.data = "estop"
        self.estop_pub.publish(msg)

    def send_teleop(self, left: float, right: float) -> None:
        msg = Float32MultiArray()
        msg.data = [left, right]
        self.teleop_pub.publish(msg)

    def _wheel_cb(self, msg: Float32MultiArray) -> None:
        self._signals.wheels_updated.emit(msg.data[0], msg.data[1])

    def _ack_cb(self, msg: String) -> None:
        self._signals.mode_confirmed.emit(msg.data)

    def _rehome_cb(self, msg: String) -> None:
        self._signals.rehome_status.emit(msg.data)

    def _tool_cb(self, msg: String) -> None:
        self._signals.tool_status.emit(msg.data)


# ===========================================================================
# Virtual twin canvas (unchanged)
# ===========================================================================

class TwinCanvas(QWidget):
    PIXELS_PER_METRE = 80

    def __init__(self, cmd_to_mps: float, wheel_base: float):
        super().__init__()
        self.setMinimumSize(480, 480)
        self.cmd_to_mps  = cmd_to_mps
        self.wheel_base  = wheel_base
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0
        self._last_update = time.monotonic()
        self._trail: list[tuple[float, float]] = []

    def reset_to_home(self) -> None:
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = math.pi / 2.0
        self._trail.clear()
        self.update()

    def update_kinematics(self, wl: float, wr: float) -> None:
        now = time.monotonic()
        dt  = min(now - self._last_update, 0.2)
        self._last_update = now

        v_left  = wl * self.cmd_to_mps
        v_right = wr * self.cmd_to_mps
        v       = (v_left + v_right) / 2.0
        omega   = (v_right - v_left) / self.wheel_base

        self.robot_theta += omega * dt
        self.robot_x     += v * math.cos(self.robot_theta) * dt
        self.robot_y     += v * math.sin(self.robot_theta) * dt

        cx, cy = self._world_to_canvas(self.robot_x, self.robot_y)
        self._trail.append((cx, cy))
        if len(self._trail) > 2000:
            self._trail.pop(0)
        self.update()

    def _world_to_canvas(self, x_m, y_m):
        cx = self.width()  / 2.0 + x_m * self.PIXELS_PER_METRE
        cy = self.height() / 2.0 - y_m * self.PIXELS_PER_METRE
        return cx, cy

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(QColor(20, 20, 20))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(self.rect())

        p.setPen(QPen(QColor(50, 50, 50), 1))
        cx0, cy0 = self._world_to_canvas(0, 0)
        for dm in range(-10, 11):
            offset = dm * self.PIXELS_PER_METRE
            p.drawLine(int(cx0 + offset), 0, int(cx0 + offset), self.height())
            p.drawLine(0, int(cy0 + offset), self.width(), int(cy0 + offset))

        p.setPen(QPen(QColor(80, 80, 80), 1, Qt.PenStyle.DashLine))
        p.drawLine(int(cx0) - 10, int(cy0), int(cx0) + 10, int(cy0))
        p.drawLine(int(cx0), int(cy0) - 10, int(cx0), int(cy0) + 10)

        if len(self._trail) >= 2:
            p.setPen(QPen(QColor(0, 180, 80), 1))
            for i in range(1, len(self._trail)):
                x0, y0 = self._trail[i - 1]
                x1, y1 = self._trail[i]
                p.drawLine(int(x0), int(y0), int(x1), int(y1))

        rx, ry = self._world_to_canvas(self.robot_x, self.robot_y)
        p.translate(rx, ry)
        p.rotate(-math.degrees(self.robot_theta))
        p.setBrush(QColor(30, 180, 30))
        p.setPen(QPen(QColor(200, 200, 200), 1))
        p.drawEllipse(-12, -12, 24, 24)
        p.setPen(QPen(QColor(255, 255, 255), 2))
        p.drawLine(0, 0, 18, 0)
        p.resetTransform()

        p.setPen(QColor(180, 180, 180))
        p.setFont(QFont("Monospace", 8))
        p.drawText(8, 16,
                   f"x={self.robot_x:+.2f}m  y={self.robot_y:+.2f}m  "
                   f"θ={math.degrees(self.robot_theta):+.1f}°")


# ===========================================================================
# D-Pad widget for teleop
# ===========================================================================

class DPadWidget(QWidget):
    """
    On-screen directional pad.

    Emits directional wheel commands while a button is held (press-and-hold
    semantics via pressed/released signals).  A 50 ms repeat timer keeps
    publishing while held.
    """

    def __init__(self, ros_node: ROS2Node):
        super().__init__()
        self._ros   = ros_node
        self._held  = (0.0, 0.0)   # currently commanded (wl, wr)

        grid = QGridLayout(self)
        grid.setSpacing(4)

        def _dpad_btn(label: str) -> QPushButton:
            btn = QPushButton(label)
            btn.setMinimumSize(52, 52)
            btn.setFont(QFont("Monospace", 16))
            btn.setStyleSheet(
                "QPushButton { background:#2a2a2a; color:white; border-radius:6px; }"
                "QPushButton:pressed { background:#3a6a3a; }"
            )
            return btn

        self.btn_fwd   = _dpad_btn("↑")
        self.btn_back  = _dpad_btn("↓")
        self.btn_left  = _dpad_btn("←")
        self.btn_right = _dpad_btn("→")
        self.btn_stop  = _dpad_btn("■")
        self.btn_stop.setToolTip("Stop (release all directions)")

        grid.addWidget(self.btn_fwd,   0, 1)
        grid.addWidget(self.btn_left,  1, 0)
        grid.addWidget(self.btn_stop,  1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_back,  2, 1)

        # Press-and-hold: set direction on press, clear on release
        self.btn_fwd.pressed.connect(
            lambda: self._set( TELEOP_DRIVE_SPEED,  TELEOP_DRIVE_SPEED))
        self.btn_back.pressed.connect(
            lambda: self._set(-TELEOP_DRIVE_SPEED, -TELEOP_DRIVE_SPEED))
        self.btn_left.pressed.connect(
            lambda: self._set(-TELEOP_TURN_SPEED,   TELEOP_TURN_SPEED))
        self.btn_right.pressed.connect(
            lambda: self._set( TELEOP_TURN_SPEED,  -TELEOP_TURN_SPEED))
        self.btn_stop.pressed.connect(
            lambda: self._set(0.0, 0.0))

        for btn in (self.btn_fwd, self.btn_back,
                    self.btn_left, self.btn_right):
            btn.released.connect(lambda: self._set(0.0, 0.0))

        # Repeat-publish at 20 Hz while a key is held
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def _set(self, wl: float, wr: float) -> None:
        self._held = (wl, wr)

    def _tick(self) -> None:
        self._ros.send_teleop(self._held[0], self._held[1])

    def stop(self) -> None:
        """Called when leaving TELEOP mode — zero the command."""
        self._set(0.0, 0.0)
        self._ros.send_teleop(0.0, 0.0)


# ===========================================================================
# Main window
# ===========================================================================

class MainWindow(QMainWindow):

    def __init__(self, ros_node: ROS2Node, signals: _Signals,
                 cmd_to_mps: float, wheel_base: float):
        super().__init__()
        self.setWindowTitle("Weeding Robot Commander GUI")
        self.ros_node = ros_node

        self.canvas = TwinCanvas(cmd_to_mps, wheel_base)

        # ---- Mode buttons ----
        self.btn_idle    = QPushButton("⏹  IDLE / STOP")
        self.btn_rehome  = QPushButton("🏠  REHOME SEQUENCE")
        self.btn_optical = QPushButton("👁  OPTICAL PATH FOLLOWING")
        self.btn_gtg     = QPushButton("🎯  GO-TO-GOAL (red only)")
        self.btn_teleop  = QPushButton("🕹  TELEOP")

        for btn in (self.btn_idle, self.btn_rehome,
                    self.btn_optical, self.btn_gtg, self.btn_teleop):
            btn.setMinimumHeight(40)

        self.btn_idle.clicked.connect(    lambda: self._request("IDLE"))
        self.btn_rehome.clicked.connect(  lambda: self._request("REHOME"))
        self.btn_optical.clicked.connect( lambda: self._request("OPTICAL"))
        self.btn_gtg.clicked.connect(     lambda: self._request("GTG"))
        self.btn_teleop.clicked.connect(  lambda: self._request("TELEOP"))

        # ---- Tool control buttons ----
        self.btn_drill_rehome = QPushButton("🔁  REHOME DRILL")
        self.btn_drill_estop  = QPushButton("🛑  E-STOP DRILL")
        self.btn_drill_rehome.setMinimumHeight(36)
        self.btn_drill_estop.setMinimumHeight(36)
        self.btn_drill_rehome.setToolTip(
            "Send 'home' to ESP32 — only works when drill is idle.")
        self.btn_drill_estop.setToolTip(
            "Immediately send estop to ESP32 regardless of state.")
        self.btn_drill_rehome.setStyleSheet(
            "QPushButton { background:#2a4a2a; color:white; }"
            "QPushButton:pressed { background:#3a7a3a; }"
        )
        self.btn_drill_estop.setStyleSheet(
            "QPushButton { background:#5a1a1a; color:white; }"
            "QPushButton:pressed { background:#aa2a2a; }"
        )
        self.btn_drill_rehome.clicked.connect(self.ros_node.send_rehome_drill)
        self.btn_drill_estop.clicked.connect( self.ros_node.send_estop_drill)

        # ---- D-Pad (teleop) ----
        self._dpad = DPadWidget(ros_node)
        self._dpad_group = QGroupBox("Teleop D-Pad")
        dpad_layout = QVBoxLayout(self._dpad_group)
        dpad_layout.addWidget(self._dpad)
        self._dpad_group.setVisible(False)   # hidden until TELEOP mode active

        # ---- Status labels ----
        self.lbl_requested  = QLabel("Requested:  IDLE")
        self.lbl_confirmed  = QLabel("Confirmed:  —")
        self.lbl_rehome_st  = QLabel("Rehome:     —")
        self.lbl_tool_st    = QLabel("Drill:      —")
        self.lbl_dr_info    = QLabel(
            f"DR ratio: {cmd_to_mps:.5f} m/s per cmd unit")

        self.lbl_gtg_hint = QLabel(
            "GTG: wheels ONLY move\nwhen red weed is detected.")
        self.lbl_gtg_hint.setFont(QFont("Monospace", 8))
        self.lbl_gtg_hint.setStyleSheet("color: #aaaaaa;")

        for lbl in (self.lbl_requested, self.lbl_confirmed,
                    self.lbl_rehome_st, self.lbl_tool_st, self.lbl_dr_info):
            lbl.setFont(QFont("Monospace", 9))

        # ---- Separator ----
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #444;")

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet("color: #444;")

        # ---- Control panel layout ----
        ctrl = QVBoxLayout()
        ctrl.addWidget(QLabel("<b>Commander Mode</b>"))
        ctrl.addWidget(self.btn_idle)
        ctrl.addWidget(self.btn_rehome)
        ctrl.addWidget(self.btn_optical)
        ctrl.addWidget(self.btn_gtg)
        ctrl.addWidget(self.btn_teleop)
        ctrl.addWidget(self.lbl_gtg_hint)
        ctrl.addSpacing(4)
        ctrl.addWidget(self._dpad_group)
        ctrl.addWidget(sep)
        ctrl.addWidget(QLabel("<b>Drill Tool</b>"))
        ctrl.addWidget(self.btn_drill_rehome)
        ctrl.addWidget(self.btn_drill_estop)
        ctrl.addWidget(sep2)
        ctrl.addSpacing(6)
        ctrl.addWidget(self.lbl_requested)
        ctrl.addWidget(self.lbl_confirmed)
        ctrl.addWidget(self.lbl_rehome_st)
        ctrl.addWidget(self.lbl_tool_st)
        ctrl.addWidget(self.lbl_dr_info)
        ctrl.addStretch()

        root = QHBoxLayout()
        root.addWidget(self.canvas, stretch=3)
        root.addLayout(ctrl,        stretch=1)

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # ---- Connect signals ----
        signals.wheels_updated.connect(self._on_wheels)
        signals.mode_confirmed.connect(self._on_mode_confirmed)
        signals.rehome_status.connect(self._on_rehome_status)
        signals.tool_status.connect(self._on_tool_status)

        # ---- Kinematics timer ----
        self._last_wl = 0.0
        self._last_wr = 0.0
        self._kin_timer = QTimer()
        self._kin_timer.timeout.connect(self._tick_kinematics)
        self._kin_timer.start(100)

    # -----------------------------------------------------------------------
    # Slots
    # -----------------------------------------------------------------------

    def _request(self, mode: str) -> None:
        # When leaving TELEOP, zero the dpad command
        if mode != "TELEOP":
            self._dpad.stop()
        self.ros_node.request_mode(mode)
        self.lbl_requested.setText(f"Requested:  {mode}")

    def _on_wheels(self, wl: float, wr: float) -> None:
        self._last_wl = wl
        self._last_wr = wr

    def _on_mode_confirmed(self, mode: str) -> None:
        self.lbl_confirmed.setText(f"Confirmed:  {mode}")

        # Show D-Pad only in TELEOP mode
        self._dpad_group.setVisible(mode == "TELEOP")

        all_btns = {
            "IDLE":    self.btn_idle,
            "REHOME":  self.btn_rehome,
            "OPTICAL": self.btn_optical,
            "GTG":     self.btn_gtg,
            "TELEOP":  self.btn_teleop,
        }
        for m, btn in all_btns.items():
            btn.setStyleSheet(
                "background-color: #2a5a2a; color: white;"
                if m == mode else ""
            )

    def _on_rehome_status(self, status: str) -> None:
        self.lbl_rehome_st.setText(f"Rehome:     {status}")
        if status == "DONE":
            self.canvas.reset_to_home()

    def _on_tool_status(self, status: str) -> None:
        colour_map = {
            "IDLE":     "#888888",
            "HOMING":   "#aaaa00",
            "DRILLING": "#aa5500",
            "DONE":     "#00aa00",
            "ERROR":    "#cc0000",
        }
        colour = colour_map.get(status.upper(), "#888888")
        self.lbl_tool_st.setText(f"Drill:      {status}")
        self.lbl_tool_st.setStyleSheet(f"color: {colour};")

    def _tick_kinematics(self) -> None:
        self.canvas.update_kinematics(self._last_wl, self._last_wr)


# ===========================================================================
# Entry point
# ===========================================================================

def _load_dead_reckoning(filepath: str):
    if os.path.exists(filepath):
        try:
            data  = np.load(filepath)
            ratio = float(data['ratio'])
            print(f"[GUI] Dead-reckoning calibration loaded: "
                  f"ratio={ratio:.6f} m/s per cmd unit")
            return ratio, DEFAULT_WHEEL_BASE
        except Exception as exc:
            print(f"[GUI] WARNING — Could not load dead-reckoning file: {exc}")
    else:
        print(f"[GUI] WARNING — Dead-reckoning file not found: {filepath}\n"
              f"       Using default ratio {DEFAULT_CMD_TO_MPS}.")
    return DEFAULT_CMD_TO_MPS, DEFAULT_WHEEL_BASE


def main(args=None):
    cal_candidates = [
        os.path.join(os.path.dirname(__file__), "dead_reckoning_cal.npz"),
        "dead_reckoning_cal.npz",
    ]
    cmd_to_mps = DEFAULT_CMD_TO_MPS
    wheel_base  = DEFAULT_WHEEL_BASE
    for path in cal_candidates:
        if os.path.exists(path):
            cmd_to_mps, wheel_base = _load_dead_reckoning(path)
            break

    rclpy.init(args=args)
    app = QApplication.instance() or QApplication(sys.argv)

    signals  = _Signals()
    ros_node = ROS2Node(signals)
    window   = MainWindow(ros_node, signals, cmd_to_mps, wheel_base)
    window.show()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    _watchdog = QTimer()
    _watchdog.timeout.connect(lambda: None)
    _watchdog.start(200)

    exit_code = app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()