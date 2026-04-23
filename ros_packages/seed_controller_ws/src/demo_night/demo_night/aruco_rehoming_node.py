import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

class RehomeNode(Node):
    def __init__(self):
        super().__init__('rehome_node')
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/vision/rehome_cmd', 10)
        
        # RealSense Pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        
        # Get Camera Intrinsics
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist_coeffs = np.zeros(5)

        # ArUco Setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.marker_length = 0.15 # 15 cm printed marker size

        # Control Parameters
        self.target_z = 1.0 # Stop 1 meter away from the wall
        self.base_speed = 35.0
        self.kp_steer = 40.0
        
        self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Rehoming Node initialized.")

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: return

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        wl, wr = 0.0, 0.0

        if ids is not None:
            # Estimate pose of the first detected marker
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            x_offset = tvec[0][0][0] # Lateral translation
            z_dist = tvec[0][0][2]   # Distance to marker
            
            error_z = z_dist - self.target_z
            error_x = x_offset
            
            if abs(error_z) > 0.05: # Drive forward/backward
                speed = self.base_speed if error_z > 0 else -self.base_speed
                steer = error_x * self.kp_steer
                wl = speed + steer
                wr = speed - steer
            else:
                self.get_logger().info("Home position reached.")
                wl, wr = 0.0, 0.0
        
        msg = Float32MultiArray()
        msg.data = [float(wl), float(wr)]
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RehomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()