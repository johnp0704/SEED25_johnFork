import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
import threading
import time

class DualCameraDisplayNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')
        self.get_logger().info("Starting High-Speed Dual Camera Node")

        # --- Shared Variables for Threads ---
        self.latest_rs_image = None
        self.latest_arducam_image = None
        self.running = True

        # --- 1. Initialize RealSense Thread ---
        self.rs_thread = threading.Thread(target=self.rs_loop, daemon=True)
        self.rs_thread.start()

        # --- 2. Initialize Arducam Thread ---
        self.arducam_index = 0
        self.arducam_thread = threading.Thread(target=self.arducam_loop, daemon=True)
        self.arducam_thread.start()

        # --- 3. Setup Display Window ---
        self.window_name = "Robot Vision: RealSense (Top) | Arducam (Bottom)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 1080) 
        
        self.timer = self.create_timer(1.0 / 30.0, self.update_display)

    def rs_loop(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        
        try:
            pipeline.start(config)
            self.get_logger().info("RealSense thread active.")
            while self.running:
                try:
                    # Extended timeout to 5000ms so it doesn't crash during initial USB handshake
                    frames = pipeline.wait_for_frames(timeout_ms=5000)
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        self.latest_rs_image = np.asanyarray(color_frame.get_data()).copy()
                except RuntimeError:
                    # Ignore temporary frame drops, keep the loop alive!
                    continue
        except Exception as e:
            self.get_logger().error(f"RealSense failed to start: {e}")
        finally:
            try:
                pipeline.stop()
            except:
                pass

    def arducam_loop(self):
        # The Jetson Hardware Acceleration Pipeline
        gstreamer_pipeline = (
            f"v4l2src device=/dev/video{self.arducam_index} ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvjpegdec ! "          # Force Jetson GPU to decode the JPEG
            "video/x-raw ! "
            "nvvidconv ! "          # Force Jetson GPU to convert the video format
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "       # Final quick software conversion to OpenCV BGR
            "video/x-raw, format=BGR ! appsink"
        )
        
        # Tell OpenCV to use the GStreamer backend instead of V4L2
        cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        # Keep the buffer limit at 1 to completely kill latency!
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f"Failed to open Arducam with GStreamer at /dev/video{self.arducam_index}.")
            return

        self.get_logger().info("Arducam hardware-accelerated thread active.")
        while self.running:
            ret, frame = cap.read()
            if ret:
                self.latest_arducam_image = frame.copy()
            else:
                time.sleep(0.01)
        cap.release()

    def update_display(self):
        rs_img = self.latest_rs_image
        ardu_img = self.latest_arducam_image
        
        # Add this temporary print statement!
        if rs_img is not None and ardu_img is not None:
            print(f"RS: {rs_img.shape} | Arducam: {ardu_img.shape}")
        #end temp print statement

        if rs_img is None:
            rs_img = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(rs_img, "RealSense: WAITING/NO SIGNAL", (350, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if ardu_img is None:
            ardu_img = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(ardu_img, "Arducam: WAITING/NO SIGNAL", (350, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if rs_img.shape[1] != ardu_img.shape[1]:
            target_width = rs_img.shape[1]
            aspect_ratio = ardu_img.shape[0] / ardu_img.shape[1]
            target_height = int(target_width * aspect_ratio)
            ardu_img = cv2.resize(ardu_img, (target_width, target_height))

        combined_image = np.vstack((rs_img, ardu_img))
        cv2.imshow(self.window_name, combined_image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.get_logger().info("Shutting down camera threads...")
        self.running = False
        time.sleep(0.5)
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