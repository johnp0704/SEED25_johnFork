from ultralytics import YOLO
import torch 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os

class MLDetectionNode(Node):
    def __init__(self):
        super().__init__('ml_red_detector_node')
        
        self.image_pub = self.create_publisher(Image, '/camera/annotated_image', 10)
        self.target_pub = self.create_publisher(Point, '/vision/target_point', 10)
        self.bridge = CvBridge()

        # Load Calibration Data
        load_file = r"calibration_data.npz"
        if os.path.exists(load_file):
            data = np.load(load_file)
            self.matrix = data['matrix']
            self.bev_width = int(data['bev_width'])
            self.bev_height = int(data['bev_height'])
        else:
            self.get_logger().error(f"Calibration file not found at {load_file}")
            self.matrix = None

        # Initialize YOLO
        model_path = r"/home/airlab/seed25/Machine_Learning/YOLOred/runs/red_train_7/weights/best.engine" 
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info("YOLO Model loaded.")

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense started.")

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image, stream=True, verbose=False, imgsz=1280)

        best_target = None
        highest_conf = 0.0

        for r in results:
            for box in r.boxes:
                conf = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                current_class = self.model.names[cls]

                # Match the standalone script logic: just check confidence!
                if conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Draw bounding box on the perspective image
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(color_image, f'{current_class} {conf}', (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    if conf > highest_conf:
                        highest_conf = conf
                        best_target = (x1, y1, x2, y2)

        # Image Transformation and Target Publishing
        if self.matrix is not None:
            # 1. Warp the currently annotated image to BEV
            bev_image = cv2.warpPerspective(color_image, self.matrix, (self.bev_width, self.bev_height))

            if best_target:
                x1, y1, x2, y2 = best_target
                u_center = (x1 + x2) / 2.0
                v_bottom = float(y2) 
                
                # Transform target point to Bird's-Eye View
                pts = np.array([[[u_center, v_bottom]]], dtype=np.float32)
                bev_pt = cv2.perspectiveTransform(pts, self.matrix)
                cX, cY = bev_pt[0][0]

                # Publish Target
                msg = Point()
                msg.x = float(cX)
                msg.y = float(cY)
                msg.z = 0.0
                self.target_pub.publish(msg)

                # Draw Target and Log to Terminal
                cv2.circle(bev_image, (int(cX), int(cY)), 12, (255, 0, 0), -1) # Blue dot on target
                self.get_logger().info(f"Red detected! Target at BEV X:{cX:.1f}, Y:{cY:.1f} | Conf: {highest_conf:.2f}")

            # 2. Resize BEV image so its height matches the color_image height (720px) to stack them cleanly
            h = color_image.shape[0]
            aspect_ratio = self.bev_width / self.bev_height
            new_w = int(h * aspect_ratio)
            bev_resized = cv2.resize(bev_image, (new_w, h))

            # 3. Stack horizontally: Left = Perspective, Right = BEV
            combined_image = np.hstack((color_image, bev_resized))
        else:
            combined_image = color_image
            if best_target:
                self.get_logger().warning("Red detected, but missing calibration matrix for BEV!")

        # Publish the combined image
        img_msg = self.bridge.cv2_to_imgmsg(combined_image, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MLDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()