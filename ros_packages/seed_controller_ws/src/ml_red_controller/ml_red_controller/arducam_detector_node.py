from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import os

# The Arducam is a standard USB camera — no RealSense SDK needed.
# It uses its own calibration file and publishes to /vision/arducam_target.

# Path to the Arducam-specific calibration file
ARDUCAM_CALIBRATION_FILE = "arducam_calibration_data.npz"

# USB camera device index — change if /dev/video0 is wrong on your system
ARDUCAM_DEVICE_INDEX = 0

# YOLO model — same engine file as RealSense detector
YOLO_MODEL_PATH = "/home/airlab/seed25/Machine_Learning/YOLOred/runs/red_train_7/weights/best.engine"

# Detection confidence threshold
CONFIDENCE_THRESH = 0.5

# Camera capture resolution
CAPTURE_WIDTH  = 1280
CAPTURE_HEIGHT = 720
CAPTURE_FPS    = 30


class ArducamDetectorNode(Node):
    def __init__(self):
        super().__init__('arducam_detector_node')

        self.image_pub  = self.create_publisher(Image, '/camera/arducam_annotated_image', 10)
        self.target_pub = self.create_publisher(Point, '/vision/arducam_target', 10)
        self.bridge = CvBridge()

        # Load Arducam-specific calibration
        self.matrix    = None
        self.bev_width = 0
        self.bev_height = 0
        self.pixels_per_meter = 1.0
        self.robot_x = 0
        self.robot_y = 0

        if os.path.exists(ARDUCAM_CALIBRATION_FILE):
            data = np.load(ARDUCAM_CALIBRATION_FILE)
            self.matrix           = data['matrix']
            self.bev_width        = int(data['bev_width'])
            self.bev_height       = int(data['bev_height'])
            self.pixels_per_meter = float(data['pixels_per_meter'])
            self.robot_x          = int(data['robot_x'])
            self.robot_y          = int(data['robot_y'])
            self.get_logger().info(f"Arducam calibration loaded from {ARDUCAM_CALIBRATION_FILE}")
        else:
            self.get_logger().error(
                f"Arducam calibration file not found at '{ARDUCAM_CALIBRATION_FILE}'. "
                f"BEV transform will be skipped — run your calibration script first."
            )

        # Load YOLO model
        self.model = YOLO(YOLO_MODEL_PATH, task='detect')
        self.get_logger().info("YOLO model loaded for Arducam.")

        # Open USB camera
        self.cap = cv2.VideoCapture(ARDUCAM_DEVICE_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"Failed to open Arducam at device index {ARDUCAM_DEVICE_INDEX}. "
                f"Check the index with: v4l2-ctl --list-devices"
            )
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS,          CAPTURE_FPS)
        self.get_logger().info(
            f"Arducam opened at device {ARDUCAM_DEVICE_INDEX} "
            f"({CAPTURE_WIDTH}x{CAPTURE_HEIGHT} @ {CAPTURE_FPS}fps)"
        )

        self.timer = self.create_timer(1.0 / CAPTURE_FPS, self.process_frame)

    def process_frame(self):
        ret, color_image = self.cap.read()
        if not ret or color_image is None:
            self.get_logger().warn("Arducam frame capture failed.", throttle_duration_sec=2.0)
            return

        results = self.model(color_image, stream=True, verbose=False, imgsz=1280)

        best_target  = None
        highest_conf = 0.0

        for r in results:
            for box in r.boxes:
                conf = math.ceil((box.conf[0] * 100)) / 100
                if conf > CONFIDENCE_THRESH:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls           = int(box.cls[0])
                    current_class = self.model.names[cls]

                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 165, 255), 3)  # orange box
                    cv2.putText(
                        color_image, f'{current_class} {conf}',
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2
                    )

                    if conf > highest_conf:
                        highest_conf = conf
                        best_target  = (x1, y1, x2, y2)

        # BEV transform and target publishing
        if self.matrix is not None:
            bev_image = cv2.warpPerspective(
                color_image, self.matrix, (self.bev_width, self.bev_height)
            )

            if best_target:
                x1, y1, x2, y2 = best_target
                u_center = (x1 + x2) / 2.0
                v_bottom = float(y2)

                pts    = np.array([[[u_center, v_bottom]]], dtype=np.float32)
                bev_pt = cv2.perspectiveTransform(pts, self.matrix)
                cX, cY = bev_pt[0][0]

                msg     = Point()
                msg.x   = float(cX)
                msg.y   = float(cY)
                msg.z   = 0.0
                self.target_pub.publish(msg)

                cv2.circle(bev_image, (int(cX), int(cY)), 12, (0, 165, 255), -1)  # orange dot
                self.get_logger().info(
                    f"[Arducam] Red detected at BEV ({cX:.1f}, {cY:.1f}) | conf={highest_conf:.2f}",
                    throttle_duration_sec=0.5
                )

            # Stack perspective + BEV side by side for display
            h            = color_image.shape[0]
            aspect_ratio = self.bev_width / self.bev_height
            new_w        = int(h * aspect_ratio)
            bev_resized  = cv2.resize(bev_image, (new_w, h))
            combined     = np.hstack((color_image, bev_resized))

        else:
            # No calibration — just publish the raw frame with annotations
            combined = color_image
            if best_target:
                self.get_logger().warn(
                    "Arducam: red detected but no calibration matrix — cannot publish BEV target.",
                    throttle_duration_sec=2.0
                )

        img_msg = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArducamDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()