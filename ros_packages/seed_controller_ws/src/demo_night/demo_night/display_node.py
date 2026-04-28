"""
display_node.py

Displays RealSense and Arducam video feeds inside a PyQt6 window using
QLabel widgets — no cv2.imshow, no OpenCV HighGUI event loop.

Bounding boxes
--------------
Both feeds draw bounding boxes on detected objects before display:
  RED  boxes — weed targets (same HSV ranges as the GTG controller).
  BLUE boxes — blue tape / path markers.

Detection is done at half resolution for speed, then bounding rects are
scaled back to full resolution.  Only the largest contour per colour is
boxed to keep it cheap.

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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

import cv2
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QLabel,
)
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject


# ===========================================================================
# Display settings
# ===========================================================================
DISPLAY_WIDTH = 640
LABEL_HEIGHT  = 20   # pixels for the coloured title bar

# ===========================================================================
# Colour detection — HSV ranges
# (must match gtg_controller_node.py for red)
# ===========================================================================

# Red wraps around hue=0/180 so we need two ranges.
LOWER_RED_1 = np.array([0,   120, 120])
UPPER_RED_1 = np.array([15,  255, 255])
LOWER_RED_2 = np.array([165, 120, 120])
UPPER_RED_2 = np.array([180, 255, 255])

# Blue tape — adjust S/V minimums if there are false positives under demo lighting.
LOWER_BLUE = np.array([100, 80, 50])
UPPER_BLUE = np.array([130, 255, 255])

# Minimum pixel area (at HALF resolution) before a detection is boxed.
MIN_RED_AREA  = 150   # ~600 px at full res
MIN_BLUE_AREA = 200   # ~800 px at full res

# Box colours in BGR
COLOR_RED_BOX  = (0,   0,   255)
COLOR_BLUE_BOX = (255, 100,   0)
BOX_THICKNESS  = 2


# ===========================================================================
# Qt signal bridge
# ===========================================================================

class _FrameSignals(QObject):
    realsense_frame = pyqtSignal(np.ndarray)
    arducam_frame   = pyqtSignal(np.ndarray)


# ===========================================================================
# Detection helper — runs on raw frames, returns annotated copy
# ===========================================================================

def _annotate_frame(frame: np.ndarray) -> np.ndarray:
    """
    Draw bounding boxes for red and blue regions.

    Strategy (cheap):
      1. Downsample to half resolution before HSV conversion.
      2. Find the single largest contour per colour (avoids iterating all).
      3. Scale the bounding rect back to full resolution.
      4. Draw on a copy of the original full-resolution frame.
    """
    out = frame.copy()

    # Downsample — INTER_NEAREST is the fastest interpolation.
    h, w = frame.shape[:2]
    small = cv2.resize(frame, (w // 2, h // 2), interpolation=cv2.INTER_NEAREST)
    hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

    # ---- Red detection ----
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1),
        cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2),
    )
    # Light morphology to kill noise (3×3 is cheap)
    red_mask = cv2.erode( red_mask, None, iterations=1)
    red_mask = cv2.dilate(red_mask, None, iterations=1)

    red_cnts, _ = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if red_cnts:
        largest = max(red_cnts, key=cv2.contourArea)
        if cv2.contourArea(largest) >= MIN_RED_AREA:
            x, y, bw, bh = cv2.boundingRect(largest)
            # Scale back to full resolution (×2)
            x, y, bw, bh = x*2, y*2, bw*2, bh*2
            cv2.rectangle(out, (x, y), (x+bw, y+bh), COLOR_RED_BOX, BOX_THICKNESS)
            cv2.putText(out, "weed", (x, max(y-6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RED_BOX, 1,
                        cv2.LINE_AA)

    # ---- Blue tape detection ----
    blue_mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
    blue_mask = cv2.erode( blue_mask, None, iterations=1)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    blue_cnts, _ = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_blue = [c for c in blue_cnts
                  if cv2.contourArea(c) >= MIN_BLUE_AREA]
    if valid_blue:
        # Pick the contour lowest in the frame (max bottom-edge Y)
        lowest = max(valid_blue,
                     key=lambda c: cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3])
        x, y, bw, bh = cv2.boundingRect(lowest)
        x, y, bw, bh = x*2, y*2, bw*2, bh*2
        cv2.rectangle(out, (x, y), (x+bw, y+bh), COLOR_BLUE_BOX, BOX_THICKNESS)
        cv2.putText(out, "tape", (x, max(y-6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BLUE_BOX, 1,
                    cv2.LINE_AA)

    return out


# ===========================================================================
# ROS2 node (runs in a background thread)
# ===========================================================================

class DisplayROSNode(Node):
    def __init__(self, signals: _FrameSignals):
        super().__init__('cam_display_node')
        self._bridge  = CvBridge()
        self._signals = signals

        self.create_subscription(Image, '/vision/realsense_display',
            self._rs_cb, SENSOR_QOS)
        self.create_subscription(Image, '/vision/arducam_display',
            self._arc_cb, SENSOR_QOS)
        self.get_logger().info("Display ROS node initialised.")

    def _rs_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._signals.realsense_frame.emit(_annotate_frame(frame))

    def _arc_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._signals.arducam_frame.emit(_annotate_frame(frame))


# ===========================================================================
# Qt window helpers
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
            "REALSENSE  |  🔴 weed  🔵 tape", "#282850", "RealSense — No Feed")
        self._arc_feed = _FeedLabel(
            "ARDUCAM    |  🔴 weed  🔵 tape", "#285028", "Arducam — No Feed")

        container = QWidget()
        layout    = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self._rs_feed)
        layout.addWidget(self._arc_feed)
        self.setCentralWidget(container)
        self.adjustSize()

        signals.realsense_frame.connect(self._rs_feed.update_frame)
        signals.arducam_frame.connect(self._arc_feed.update_frame)


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    app = QApplication.instance() or QApplication(sys.argv)

    signals  = _FrameSignals()
    ros_node = DisplayROSNode(signals)
    window   = DisplayWindow(signals)
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