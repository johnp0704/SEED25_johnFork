import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Display dimensions — reduced to 640 to save CPU cycles on the robot
DISPLAY_WIDTH = 640

class DisplayNode(Node):
    def __init__(self):
        super().__init__('cam_display_node')
        self.bridge = CvBridge()

        self.realsense_frame = None
        self.arducam_frame   = None
        self.needs_render    = False

        self.window_name = "Weeding Robot Vision Feed"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # Subscribing to the new dedicated display topics
        self.create_subscription(
            Image, '/vision/realsense_display',
            self.realsense_callback, 2
        )
        self.create_subscription(
            Image, '/vision/arducam_display',
            self.arducam_callback, 2
        )

        # Check for updates at 30Hz, but only render if flagged
        self.create_timer(1.0 / 30.0, self.render_loop)
        self.get_logger().info("Lightweight Display Node Initialized.")

    def realsense_callback(self, msg):
        self.realsense_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.needs_render = True

    def arducam_callback(self, msg):
        self.arducam_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.needs_render = True

    def _resize_to_width(self, img, width):
        """Fast resize using nearest-neighbor interpolation."""
        h, w = img.shape[:2]
        new_h = int(h * width / w)
        return cv2.resize(img, (width, new_h), interpolation=cv2.INTER_NEAREST)

    def _make_placeholder(self, label):
        """Black placeholder frame."""
        ph = np.zeros((240, DISPLAY_WIDTH, 3), dtype=np.uint8)
        cv2.putText(
            ph, label, (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2
        )
        return ph

    def render_loop(self):
        if not self.needs_render:
            return

        top = (
            self._resize_to_width(self.realsense_frame, DISPLAY_WIDTH)
            if self.realsense_frame is not None
            else self._make_placeholder("RealSense — No Feed")
        )
        bottom = (
            self._resize_to_width(self.arducam_frame, DISPLAY_WIDTH)
            if self.arducam_frame is not None
            else self._make_placeholder("Arducam — No Feed")
        )

        label_h = 24
        rs_bar  = np.full((label_h, DISPLAY_WIDTH, 3), (40, 40, 160), dtype=np.uint8)
        ar_bar  = np.full((label_h, DISPLAY_WIDTH, 3), (40, 120, 40), dtype=np.uint8)
        cv2.putText(rs_bar, "REALSENSE", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(ar_bar, "ARDUCAM",   (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        combined = np.vstack((rs_bar, top, ar_bar, bottom))
        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)
        
        self.needs_render = False

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