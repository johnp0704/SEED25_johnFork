import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# Display dimensions — each camera feed is resized to this width before stacking
DISPLAY_WIDTH = 1280


class DisplayNode(Node):
    def __init__(self):
        super().__init__('cam_display_node')
        self.bridge = CvBridge()

        self.realsense_frame = None
        self.arducam_frame   = None

        self.window_name = "Vision Feed — RealSense (top) | Arducam (bottom)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, DISPLAY_WIDTH, 480)

        self.create_subscription(
            Image, '/camera/annotated_image',
            self.realsense_callback, 10
        )
        self.create_subscription(
            Image, '/camera/arducam_annotated_image',
            self.arducam_callback, 10
        )

        # Render at 30Hz regardless of camera callback rate
        self.create_timer(1.0 / 30.0, self.render)

    def realsense_callback(self, msg):
        self.realsense_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def arducam_callback(self, msg):
        self.arducam_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _resize_to_width(self, img, width):
        """Resize image to a fixed width, preserving aspect ratio."""
        h, w = img.shape[:2]
        new_h = int(h * width / w)
        return cv2.resize(img, (width, new_h))

    def _make_placeholder(self, label):
        """Black placeholder frame shown when a camera has no feed yet."""
        ph = np.zeros((240, DISPLAY_WIDTH, 3), dtype=np.uint8)
        cv2.putText(
            ph, label, (20, 130),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 100), 2
        )
        return ph

    def render(self):
        top = (
            self._resize_to_width(self.realsense_frame, DISPLAY_WIDTH)
            if self.realsense_frame is not None
            else self._make_placeholder("RealSense — no feed")
        )
        bottom = (
            self._resize_to_width(self.arducam_frame, DISPLAY_WIDTH)
            if self.arducam_frame is not None
            else self._make_placeholder("Arducam — no feed")
        )

        # Add a thin colored label bar to each feed so they're easy to tell apart
        label_h = 28
        rs_bar  = np.full((label_h, DISPLAY_WIDTH, 3), (40, 40, 160), dtype=np.uint8)  # dark red
        ar_bar  = np.full((label_h, DISPLAY_WIDTH, 3), (40, 120, 40), dtype=np.uint8)  # dark green
        cv2.putText(rs_bar, "REALSENSE", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        cv2.putText(ar_bar, "ARDUCAM",   (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)

        combined = np.vstack((rs_bar, top, ar_bar, bottom))
        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()