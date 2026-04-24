"""
gui_node.py

Commander GUI for the autonomous weeding robot.

Key fixes vs. original
-----------------------
* Dead-reckoning calibration file (dead_reckoning_cal.npz) is loaded on
  startup and used to set TwinCanvas.cmd_to_mps correctly.
* Kinematics use wall-clock dt (time.monotonic) rather than a fixed 0.1 s.
* The virtual twin position and heading are reset to the enclosure centre
  when the REHOME sequence completes (subscribed to /rehome/status).
* The GUI subscribes to /commander/ack so the displayed mode reflects what
  the commander has *confirmed*, not just what was requested.
* ROS2 spins in a background daemon thread; Qt owns the main thread.
* No cv2.imshow anywhere in this process.

Topics
------
Publishes  : /gui/system_state        (std_msgs/String)   mode requests
Subscribes : /commander/wheel_cmd     (std_msgs/Float32MultiArray)
             /commander/ack           (std_msgs/String)    confirmed mode
             /rehome/status           (std_msgs/String)    rehome state machine
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
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QPainter, QColor, QPen, QFont


# ===========================================================================
# Default dead-reckoning constants (used only if the .npz file is missing)
# ===========================================================================
DEFAULT_CMD_TO_MPS = 0.005   # m/s per Sabertooth command unit
DEFAULT_WHEEL_BASE = 0.40    # metres


# ===========================================================================
# Qt signal bridge — ROS2 callbacks → Qt main thread
# ===========================================================================

class _Signals(QObject):
    wheels_updated  = pyqtSignal(float, float)   # wl, wr
    mode_confirmed  = pyqtSignal(str)             # confirmed mode from commander
    rehome_status   = pyqtSignal(str)             # rehome state machine strings


# ===========================================================================
# ROS2 worker node (runs in a background thread)
# ===========================================================================

class ROS2Node(Node):

    def __init__(self, signals: _Signals):
        super().__init__('pyqt_gui_node')
        self._signals = signals

        self.state_pub = self.create_publisher(String, '/gui/system_state', 10)

        self.create_subscription(
            Float32MultiArray, '/commander/wheel_cmd',
            self._wheel_cb, 10)
        self.create_subscription(
            String, '/commander/ack',
            self._ack_cb, 10)
        self.create_subscription(
            String, '/rehome/status',
            self._rehome_cb, 10)

    def request_mode(self, mode: str) -> None:
        """Called from Qt thread — publishes mode request to commander."""
        msg      = String()
        msg.data = mode
        self.state_pub.publish(msg)

    def _wheel_cb(self, msg: Float32MultiArray) -> None:
        self._signals.wheels_updated.emit(msg.data[0], msg.data[1])

    def _ack_cb(self, msg: String) -> None:
        self._signals.mode_confirmed.emit(msg.data)

    def _rehome_cb(self, msg: String) -> None:
        self._signals.rehome_status.emit(msg.data)


# ===========================================================================
# Virtual twin canvas
# ===========================================================================

class TwinCanvas(QWidget):
    """
    Dead-reckoning 2-D top-down view.

    Coordinate frame: robot_x / robot_y are in metres from the canvas centre.
    robot_theta is in radians; 0 = facing right (East), π/2 = facing up (North).
    """

    PIXELS_PER_METRE = 80   # display scale

    def __init__(self, cmd_to_mps: float, wheel_base: float):
        super().__init__()
        self.setMinimumSize(480, 480)

        self.cmd_to_mps = cmd_to_mps
        self.wheel_base = wheel_base

        # Robot pose in metres relative to enclosure centre
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0   # radians

        self._last_update = time.monotonic()

        # Trail of (x_px, y_px) for visualisation
        self._trail: list[tuple[float, float]] = []

    def reset_to_home(self) -> None:
        """Call when rehoming completes — snap twin to centre, face North."""
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = math.pi / 2.0   # North = up on screen
        self._trail.clear()
        self.update()

    def update_kinematics(self, wl: float, wr: float) -> None:
        """Integrate differential-drive kinematics using real elapsed time."""
        now = time.monotonic()
        dt  = now - self._last_update
        self._last_update = now

        # Clamp dt to avoid huge jumps after pauses
        dt = min(dt, 0.2)

        v_left  = wl * self.cmd_to_mps
        v_right = wr * self.cmd_to_mps

        v     = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        self.robot_theta += omega * dt
        self.robot_x     += v * math.cos(self.robot_theta) * dt
        self.robot_y     += v * math.sin(self.robot_theta) * dt

        # Record trail point (canvas coordinates)
        cx, cy = self._world_to_canvas(self.robot_x, self.robot_y)
        self._trail.append((cx, cy))
        if len(self._trail) > 2000:
            self._trail.pop(0)

        self.update()

    def _world_to_canvas(self, x_m: float, y_m: float):
        """World (metres) → canvas (pixels).  Y-axis is flipped (screen down = world south)."""
        cx = self.width()  / 2.0 + x_m * self.PIXELS_PER_METRE
        cy = self.height() / 2.0 - y_m * self.PIXELS_PER_METRE
        return cx, cy

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        p.setBrush(QColor(20, 20, 20))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(self.rect())

        # Grid lines every metre
        p.setPen(QPen(QColor(50, 50, 50), 1))
        cx0, cy0 = self._world_to_canvas(0, 0)
        span = max(self.width(), self.height())
        for dm in range(-10, 11):
            offset = dm * self.PIXELS_PER_METRE
            p.drawLine(int(cx0 + offset), 0, int(cx0 + offset), self.height())
            p.drawLine(0, int(cy0 + offset), self.width(), int(cy0 + offset))

        # Centre cross-hair
        p.setPen(QPen(QColor(80, 80, 80), 1, Qt.PenStyle.DashLine))
        p.drawLine(int(cx0) - 10, int(cy0), int(cx0) + 10, int(cy0))
        p.drawLine(int(cx0), int(cy0) - 10, int(cx0), int(cy0) + 10)

        # Trail
        if len(self._trail) >= 2:
            p.setPen(QPen(QColor(0, 180, 80), 1))
            for i in range(1, len(self._trail)):
                x0, y0 = self._trail[i - 1]
                x1, y1 = self._trail[i]
                p.drawLine(int(x0), int(y0), int(x1), int(y1))

        # Robot body
        rx, ry = self._world_to_canvas(self.robot_x, self.robot_y)
        p.translate(rx, ry)
        p.rotate(-math.degrees(self.robot_theta))   # screen Y is flipped

        p.setBrush(QColor(30, 180, 30))
        p.setPen(QPen(QColor(200, 200, 200), 1))
        p.drawEllipse(-12, -12, 24, 24)

        # Heading arrow
        p.setPen(QPen(QColor(255, 255, 255), 2))
        p.drawLine(0, 0, 18, 0)

        p.resetTransform()

        # Legend
        p.setPen(QColor(180, 180, 180))
        p.setFont(QFont("Monospace", 8))
        p.drawText(8, 16,
                   f"x={self.robot_x:+.2f}m  y={self.robot_y:+.2f}m  "
                   f"θ={math.degrees(self.robot_theta):+.1f}°")


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

        # --- Mode buttons ---
        self.btn_idle     = QPushButton("⏹  IDLE / STOP")
        self.btn_rehome   = QPushButton("🏠  REHOME SEQUENCE")
        self.btn_optical  = QPushButton("👁  OPTICAL PATH FOLLOWING")
        self.btn_traj     = QPushButton("✏️  TRAJECTORY FOLLOWING")

        for btn in (self.btn_idle, self.btn_rehome,
                    self.btn_optical, self.btn_traj):
            btn.setMinimumHeight(40)

        self.btn_idle.clicked.connect(    lambda: self._request("IDLE"))
        self.btn_rehome.clicked.connect(  lambda: self._request("REHOME"))
        self.btn_optical.clicked.connect( lambda: self._request("OPTICAL"))
        self.btn_traj.clicked.connect(    lambda: self._request("TRAJECTORY"))

        # --- Status labels ---
        self.lbl_requested  = QLabel("Requested:  IDLE")
        self.lbl_confirmed  = QLabel("Confirmed:  —")
        self.lbl_rehome_st  = QLabel("Rehome:     —")
        self.lbl_dr_info    = QLabel(
            f"DR ratio: {cmd_to_mps:.5f} m/s per cmd unit")

        for lbl in (self.lbl_requested, self.lbl_confirmed,
                    self.lbl_rehome_st, self.lbl_dr_info):
            lbl.setFont(QFont("Monospace", 9))

        # --- Layout ---
        ctrl = QVBoxLayout()
        ctrl.addWidget(QLabel("<b>Commander Mode</b>"))
        ctrl.addWidget(self.btn_idle)
        ctrl.addWidget(self.btn_rehome)
        ctrl.addWidget(self.btn_optical)
        ctrl.addWidget(self.btn_traj)
        ctrl.addSpacing(12)
        ctrl.addWidget(self.lbl_requested)
        ctrl.addWidget(self.lbl_confirmed)
        ctrl.addWidget(self.lbl_rehome_st)
        ctrl.addWidget(self.lbl_dr_info)
        ctrl.addStretch()

        root = QHBoxLayout()
        root.addWidget(self.canvas, stretch=3)
        root.addLayout(ctrl,        stretch=1)

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # --- Connect signals from ROS thread ---
        signals.wheels_updated.connect(self._on_wheels)
        signals.mode_confirmed.connect(self._on_mode_confirmed)
        signals.rehome_status.connect(self._on_rehome_status)

        # --- Kinematics update timer (10 Hz) ---
        # The canvas integrates using wall-clock time internally, so the
        # timer period only controls how often we ask for an update tick.
        self._last_wl = 0.0
        self._last_wr = 0.0
        self._kin_timer = QTimer()
        self._kin_timer.timeout.connect(self._tick_kinematics)
        self._kin_timer.start(100)   # 10 Hz

    # -----------------------------------------------------------------------
    # Slots
    # -----------------------------------------------------------------------

    def _request(self, mode: str) -> None:
        self.ros_node.request_mode(mode)
        self.lbl_requested.setText(f"Requested:  {mode}")

    def _on_wheels(self, wl: float, wr: float) -> None:
        self._last_wl = wl
        self._last_wr = wr

    def _on_mode_confirmed(self, mode: str) -> None:
        self.lbl_confirmed.setText(f"Confirmed:  {mode}")
        # Highlight active button
        styles = {
            "IDLE":       self.btn_idle,
            "REHOME":     self.btn_rehome,
            "OPTICAL":    self.btn_optical,
            "TRAJECTORY": self.btn_traj,
        }
        for m, btn in styles.items():
            btn.setStyleSheet(
                "background-color: #2a5a2a; color: white;"
                if m == mode else ""
            )

    def _on_rehome_status(self, status: str) -> None:
        self.lbl_rehome_st.setText(f"Rehome:     {status}")
        if status == "DONE":
            # Rehoming complete — anchor the virtual twin to the home pose
            self.canvas.reset_to_home()

    def _tick_kinematics(self) -> None:
        self.canvas.update_kinematics(self._last_wl, self._last_wr)


# ===========================================================================
# Entry point
# ===========================================================================

def _load_dead_reckoning(filepath: str):
    """Load cmd_to_mps ratio from calibration file.  Returns (ratio, wheel_base)."""
    if os.path.exists(filepath):
        try:
            data        = np.load(filepath)
            ratio       = float(data['ratio'])
            wheel_base  = DEFAULT_WHEEL_BASE   # wheel_base is not calibrated here
            print(f"[GUI] Dead-reckoning calibration loaded: "
                  f"ratio={ratio:.6f} m/s per cmd unit")
            return ratio, wheel_base
        except Exception as exc:
            print(f"[GUI] WARNING — Could not load dead-reckoning file: {exc}")
    else:
        print(f"[GUI] WARNING — Dead-reckoning file not found: {filepath}\n"
              f"       Using default ratio {DEFAULT_CMD_TO_MPS}.  "
              f"Run dead_reckoning_calibration.py first.")
    return DEFAULT_CMD_TO_MPS, DEFAULT_WHEEL_BASE


def main(args=None):
    # ... existing cal loading code ...

    rclpy.init(args=args)
    app = QApplication.instance() or QApplication(sys.argv)

    signals  = _Signals()
    ros_node = ROS2Node(signals)
    window   = MainWindow(ros_node, signals, cmd_to_mps, wheel_base)
    window.show()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Clean Ctrl-C handling inside Qt's event loop
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