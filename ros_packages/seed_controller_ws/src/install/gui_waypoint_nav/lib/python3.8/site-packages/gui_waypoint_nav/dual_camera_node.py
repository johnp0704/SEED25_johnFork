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
        self.get_logger().info("Starting High-Speed Dual Camera Node (Scaled Display)")

        self.latest_rs_image = None
        self.latest_arducam_image = None
        self.running = True

        # --- 1. Initialize Threads ---
        self.rs_thread = threading.Thread(target=self.rs_loop, daemon=True)
        self.rs_thread.start()

        self.arducam_index = 0
        self.arducam_thread = threading.Thread(target=self.arducam_loop, daemon=True)
        self.arducam_thread.start()

        # --- 2. Setup Display Window ---
        self.window_name = "Robot Vision: RealSense (Top) | Arducam (Bottom)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE) # Let OpenCV handle the window size natively
        
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
                    frames = pipeline.wait_for_frames(timeout_ms=5000)
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        self.latest_rs_image = np.asanyarray(color_frame.get_data()).copy()
                except RuntimeError:
                    continue
        except Exception as e:
            self.get_logger().error(f"RealSense failed to start: {e}")
        finally:
            try:
                pipeline.stop()
            except:
                pass

    def arducam_loop(self):
        # Revert to the proven V4L2 method from your test script
        cap = cv2.VideoCapture(self.arducam_index, cv2.CAP_V4L2)
        
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f"Failed to open Arducam at /dev/video{self.arducam_index}.")
            return

        self.get_logger().info("Arducam thread active.")
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

        # -- Handle Missing Signals --
        if rs_img is None:
            rs_img = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(rs_img, "RealSense: WAITING/NO SIGNAL", (300, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if ardu_img is None:
            ardu_img = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(ardu_img, "Arducam: WAITING/NO SIGNAL", (300, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        # -- 75% Data Reduction Strategy (Scale down for GUI) --
        # We resize the 1280x720 images down to 640x360 before stacking.
        # This saves the Jetson CPU from rendering a massive window.
        rs_img_small = cv2.resize(rs_img, (640, 360))
        ardu_img_small = cv2.resize(ardu_img, (640, 360))

        # -- Stack and Display --
        combined_image = np.vstack((rs_img_small, ardu_img_small))
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