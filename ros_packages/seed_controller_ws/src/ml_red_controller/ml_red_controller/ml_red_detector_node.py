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
from ultralytics import YOLO

class MLDetectionNode(Node):
    def __init__(self):
        super().__init__('ml_detector_node')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/annotated_image', 10)
        self.target_pub = self.create_publisher(Point, '/vision/target_point', 10)
        self.bridge = CvBridge()

        # Load Calibration Data (for perspective transform)
        load_file = r"calibration_data.npz"
        if os.path.exists(load_file):
            data = np.load(load_file)
            self.matrix = data['matrix']
        else:
            self.get_logger().error(f"Calibration file not found at {load_file}")
            self.matrix = None

        # Initialize YOLO
        model_path = r"C:\UVM\SEED\SEED25\Machine_Learning\YOLOred\runs\red_train_7\weights\best.pt"
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO Model loaded.")

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense started.")

        # Main loop timer (runs at roughly 30Hz to match camera)
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image, stream=True, verbose=False)

        best_target = None
        highest_conf = 0.0

        for r in results:
            for box in r.boxes:
                conf = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                current_class = self.model.names[cls]

                if current_class == "Red" and conf > 0.8:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Draw bounding box on the image
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(color_image, f'{current_class} {conf}', (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # Keep track of the highest confidence detection
                    if conf > highest_conf:
                        highest_conf = conf
                        best_target = (x1, y1, x2, y2)

        # Calculate BEV target and publish
        if best_target and self.matrix is not None:
            x1, y1, x2, y2 = best_target
            # Bottom center of the bounding box represents the footprint on the ground
            u_center = (x1 + x2) / 2.0
            v_bottom = float(y2) 
            
            # Transform perspective point to Bird's-Eye View
            pts = np.array([[[u_center, v_bottom]]], dtype=np.float32)
            bev_pt = cv2.perspectiveTransform(pts, self.matrix)
            cX, cY = bev_pt[0][0]

            msg = Point()
            msg.x = float(cX)
            msg.y = float(cY)
            msg.z = 0.0 # Z is unused here
            self.target_pub.publish(msg)

        # Publish annotated image to ROS
        img_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
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