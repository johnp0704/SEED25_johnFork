import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np

class DualCameraDisplayNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')
        self.get_logger().info("Starting Dual Camera Display Node")

        # --- 1. Initialize RealSense (Top Camera) ---
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        
        try:
            self.rs_pipeline.start(self.rs_config)
            self.get_logger().info("RealSense started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")

        # --- 2. Initialize Arducam (Bottom Camera) ---
        # 0 is usually the default index for the first standard USB camera.
        self.arducam_index = 0 
        
        # On a Jetson, using CAP_V4L2 directly interfaces with the Linux video drivers
        self.cap = cv2.VideoCapture(self.arducam_index, cv2.CAP_V4L2)
        
        # Request 720p to match the RealSense and save USB bandwidth
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open Arducam at /dev/video{self.arducam_index}")
        else:
            self.get_logger().info("Arducam started successfully.")

        # --- 3. Setup Display Window ---
        self.window_name = "Robot Vision: RealSense (Top) | Arducam (Bottom)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # Resize the window so it doesn't take up the entire monitor initially
        cv2.resizeWindow(self.window_name, 960, 1080) 
        
        # Timer to fetch and display frames at roughly 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.process_frames)

    def process_frames(self):
        rs_image = None
        arducam_image = None

        # -- Fetch RealSense Frame --
        try:
            frames = self.rs_pipeline.wait_for_frames(timeout_ms=100)
            color_frame = frames.get_color_frame()
            if color_frame:
                rs_image = np.asanyarray(color_frame.get_data())
        except RuntimeError:
            pass # Ignore temporary timeouts

        # -- Fetch Arducam Frame --
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                arducam_image = frame

        # -- Handle Missing Signals Gracefully --
        if rs_image is None:
            rs_image = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(rs_image, "RealSense: NO SIGNAL", (400, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

        if arducam_image is None:
            arducam_image = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(arducam_image, "Arducam: NO SIGNAL", (400, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

        # -- Match Widths for Stacking (Just in case the Arducam rejects the 720p request) --
        if rs_image.shape[1] != arducam_image.shape[1]:
            target_width = rs_image.shape[1]
            aspect_ratio = arducam_image.shape[0] / arducam_image.shape[1]
            target_height = int(target_width * aspect_ratio)
            arducam_image = cv2.resize(arducam_image, (target_width, target_height))

        # -- Stack and Display --
        combined_image = np.vstack((rs_image, arducam_image))
        cv2.imshow(self.window_name, combined_image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.get_logger().info("Shutting down cameras...")
        try:
            self.rs_pipeline.stop()
        except:
            pass
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()