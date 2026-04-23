"""
display_node.py

Displays RealSense and Arducam video feeds inside a PyQt6 window using
QLabel widgets — no cv2.imshow, no OpenCV HighGUI event loop.

Running this alongside the PyQt6 GUI node (gui_node.py) is safe because
both use the same Qt event loop.  Launch them from the SAME process by
importing and composing them, or run as separate processes (recommended
for ROS2 component isolation).

Topics subscribed
-----------------
/vision/realsense_display  (sensor_msgs/Image, bgr8)
/vision/arducam_display    (sensor_msgs/Image, bgr8)
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QLabel,
)
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject


# ===========================================================================
# Target display width per feed (pixels).  Height is scaled to preserve AR.
# ===========================================================================
DISPLAY_WIDTH = 640
LABEL_HEIGHT  = 20   # pixels for the coloured title bar


# ===========================================================================
# Qt signal bridge — lets the ROS2 callback (any thread) hand a numpy array
# to the Qt main thread without a direct call.
# ===========================================================================

class _FrameSignals(QObject):
    realsense_frame = pyqtSignal(np.ndarray)
    arducam_frame   = pyqtSignal(np.ndarray)


# ===========================================================================
# ROS2 node (runs in a background thread)
# ===========================================================================

class DisplayROSNode(Node):
    def __init__(self, signals: _FrameSignals):
        super().__init__('cam_display_node')
        self._bridge  = CvBridge()
        self._signals = signals

        self.create_subscription(
            Image, '/vision/realsense_display',
            self._rs_cb, 2,
        )
        self.create_subscription(
            Image, '/vision/arducam_display',
            self._arc_cb, 2,
        )
        self.get_logger().info("Display ROS node initialised.")

    def _rs_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._signals.realsense_frame.emit(frame)

    def _arc_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._signals.arducam_frame.emit(frame)


# ===========================================================================
# Qt window
# ===========================================================================

def _bgr_to_pixmap(frame: np.ndarray, target_width: int) -> QPixmap:
    """Convert a BGR numpy frame to a QPixmap scaled to target_width."""
    h, w = frame.shape[:2]
    new_h = int(h * target_width / w)
    resized = cv2.resize(frame, (target_width, new_h),
                         interpolation=cv2.INTER_NEAREST)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    qimg = QImage(
        rgb.data,
        rgb.shape[1], rgb.shape[0],
        rgb.strides[0],
        QImage.Format.Format_RGB888,
    )
    return QPixmap.fromImage(qimg)


def _placeholder_pixmap(label_text: str, target_width: int) -> QPixmap:
    """Black placeholder frame with centred grey text."""
    ph = np.zeros((240, target_width, 3), dtype=np.uint8)
    cv2.putText(
        ph, label_text,
        (20, 130), cv2.FONT_HERSHEY_SIMPLEX,
        0.8, (100, 100, 100), 2,
    )
    rgb = cv2.cvtColor(ph, cv2.COLOR_BGR2RGB)
    qimg = QImage(
        rgb.data,
        rgb.shape[1], rgb.shape[0],
        rgb.strides[0],
        QImage.Format.Format_RGB888,
    )
    return QPixmap.fromImage(qimg)


class _FeedLabel(QWidget):
    """A titled camera feed widget: coloured header bar + image QLabel."""

    def __init__(self, title: str, header_colour: str, placeholder: str):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Coloured title bar
        header = QLabel(title)
        header.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        header.setFixedHeight(LABEL_HEIGHT)
        header.setStyleSheet(
            f"background-color: {header_colour}; color: white; "
            f"padding-left: 6px; font-weight: bold;"
        )
        layout.addWidget(header)

        # Image display
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        self.image_label.setPixmap(
            _placeholder_pixmap(placeholder, DISPLAY_WIDTH))
        layout.addWidget(self.image_label)

    def update_frame(self, frame: np.ndarray) -> None:
        self.image_label.setPixmap(_bgr_to_pixmap(frame, DISPLAY_WIDTH))


class DisplayWindow(QMainWindow):
    def __init__(self, signals: _FrameSignals):
        super().__init__()
        self.setWindowTitle("Weeding Robot Vision Feed")

        self._rs_feed  = _FeedLabel(
            "REALSENSE", "#282850", "RealSense — No Feed")
        self._arc_feed = _FeedLabel(
            "ARDUCAM",   "#285028", "Arducam — No Feed")

        container = QWidget()
        layout    = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._rs_feed)
        layout.addWidget(self._arc_feed)
        self.setCentralWidget(container)
        self.adjustSize()

        # Connect signals from the ROS2 thread to Qt slots (thread-safe)
        signals.realsense_frame.connect(self._rs_feed.update_frame)
        signals.arducam_frame.connect(self._arc_feed.update_frame)


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    app = QApplication.instance() or QApplication(sys.argv)

    signals = _FrameSignals()
    ros_node = DisplayROSNode(signals)
    window   = DisplayWindow(signals)
    window.show()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Install a signal handler so Ctrl-C on the terminal kills Qt cleanly
    import signal
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    # A short QTimer lets Python's signal handler actually fire inside Qt's loop
    _watchdog = QTimer()
    _watchdog.timeout.connect(lambda: None)
    _watchdog.start(200)

    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()