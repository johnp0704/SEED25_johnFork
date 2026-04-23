import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import pyrealsense2 as rs
from ml_red_controller.PID import PID

class OpticalPathNode(Node):
    def __init__(self):
        super().__init__('optical_path_node')
        
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/vision/optical_cmd', 10)
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.pid_steer = PID(Kp=0.2, Ki=0.01, Kd=0.05, N=10, Ts=1/30.0, umax=30.0, umin=-30.0)
        self.base_speed = 35.0

        # Run the RealSense Sampler script to refine these HSV values for your blue tape
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([140, 255, 255])

        self.create_timer(1/30.0, self.control_loop)
        self.get_logger().info("Optical Path Tracking initialized.")

    def control_loop(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: return

        frame = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Focus on the bottom half of the image where the line is
        h, w = mask.shape
        mask[0:int(h/2), :] = 0 

        M = cv2.moments(mask)
        wl, wr = 0.0, 0.0

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            error_x = (w / 2) - cx # Positive if line is to the left
            
            correction = self.pid_steer.update(setpoint=0.0, output=-error_x)
            
            wl = self.base_speed - correction
            wr = self.base_speed + correction
        else:
            # Line lost, halt or execute recovery spin
            wl, wr = 0.0, 0.0

        msg = Float32MultiArray()
        msg.data = [float(wl), float(wr)]
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpticalPathNode()
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