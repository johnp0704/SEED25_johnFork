import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import math
import os
import cv2
import pyrealsense2 as rs
from ml_red_controller.PID import PID
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ==============================================================================
# TUNING CONSTANTS
# ==============================================================================

# --- RealSense offset corrections (meters) ---
REALSENSE_OFFSET_X = -0.3
REALSENSE_OFFSET_Y =  0.16

# --- Arducam offset corrections (meters) ---
ARDUCAM_OFFSET_X = 0.1
ARDUCAM_OFFSET_Y = 0.0

# --- Camera handoff ---
REALSENSE_TIMEOUT_SEC = 0.5

# --- Goal threshold: stop and trigger auger within this distance (meters) ---
GOAL_THRESH_REALSENSE = 0.30
GOAL_THRESH_ARDUCAM   = 0.15

# --- Motion parameters ---
MAX_ACTUATOR_INPUT = 50.0
PIVOT_THRESH       = np.deg2rad(30.0)
PIVOT_SPEED        = 40.0
DRIVE_SPEED        = 40.0

# --- PID steering (only active during DRIVE state) ---
PID_KP  = 15.0
PID_KI  = 0.5
PID_KD  = 2.0
PID_N   = 15.0
PID_KAW = 1.0

# --- HSV Masking for Red ---
# Note: Red hue wraps around the 180 mark in OpenCV's HSV space, 
# so we need two ranges. Update these with your sampler script values.
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])

LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

# Minimum pixel area to consider a valid target (filters out background noise)
MIN_CONTOUR_AREA = 500

# ==============================================================================

class GTGControllerNode(Node):
    def __init__(self):
        super().__init__('gtg_controller_node')

        self.load_calibration()

        self.active_camera       = 'realsense'
        self.last_realsense_time = 0

        self.pid_steer = PID(
            Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
            N=PID_N, Ts=1/30.0,
            umax=MAX_ACTUATOR_INPUT, umin=-MAX_ACTUATOR_INPUT,
            Kaw=PID_KAW
        )

        self.wheel_cmd_pub     = self.create_publisher(Float32MultiArray, '/vision/gtg_cmd', 10)
        self.auger_trigger_pub = self.create_publisher(String, '/auger/activate', 10)

        self.bridge = CvBridge()
        self.rs_display_pub  = self.create_publisher(Image, '/vision/realsense_display', 2)
        self.arc_display_pub = self.create_publisher(Image, '/vision/arducam_display', 2)

        # Initialize RealSense Pipeline
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            self.get_logger().info("RealSense pipeline started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")

        # Initialize Arducam (Assuming standard V4L2 device at index 0)
        # Update the index (0, 1, 2) based on your USB device tree
        self.cap_arducam = cv2.VideoCapture(0)
        if self.cap_arducam.isOpened():
            self.get_logger().info("Arducam initialized.")
        else:
            self.get_logger().warn("Arducam not found or failed to open.")

        # Master Vision Loop (30 Hz)
        self.create_timer(1/30.0, self.process_vision)

    def load_calibration(self):
        rs_file  = "/home/airlab/seed25/ros_packages/seed_controller_ws/src/ml_red_controller/ml_red_controller/calibration_data.npz"
        arc_file = "/home/airlab/seed25/ros_packages/seed_controller_ws/src/ml_red_controller/ml_red_controller/arducam_calibration_data.npz"

        if os.path.exists(rs_file):
            data = np.load(rs_file)
            self.rs_pixels_per_meter = float(data['pixels_per_meter'])
            self.rs_robot_x          = int(data['robot_x'])
            self.rs_robot_y          = int(data['robot_y'])
            self.get_logger().info("RealSense calibration loaded.")
        else:
            self.get_logger().error(f"RealSense calibration not found: {rs_file}")
            self.rs_pixels_per_meter = 1.0
            self.rs_robot_x = 0
            self.rs_robot_y = 0

        if os.path.exists(arc_file):
            data = np.load(arc_file)
            self.arc_pixels_per_meter = float(data['pixels_per_meter'])
            self.arc_robot_x          = int(data['robot_x'])
            self.arc_robot_y          = int(data['robot_y'])
            self.get_logger().info("Arducam calibration loaded.")
        else:
            self.get_logger().warn(f"Arducam calibration not found: {arc_file}")
            self.arc_pixels_per_meter = 1.0
            self.arc_robot_x = 0
            self.arc_robot_y = 0

    def _reset_pid(self):
        self.pid_steer.istate     = 0.0
        self.pid_steer.dstate     = 0.0
        self.pid_steer.error_prev = 0.0

    def _pixel_to_robot_frame(self, cX, cY, offset_x, offset_y, pixels_per_meter, robot_x, robot_y):
        dx_px   = cX - robot_x
        dy_px   = robot_y - cY
        rel_x   = (dx_px / pixels_per_meter) + offset_x
        rel_y   = (dy_px / pixels_per_meter) + offset_y
        x_robot =  rel_y
        y_robot = -rel_x
        return x_robot, y_robot

    def get_red_centroid(self, frame):
        """Extracts the largest red contour from an image and returns its centroid."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Combine the two red masks
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Calculate moments
        M = cv2.moments(mask)
        if M["m00"] > MIN_CONTOUR_AREA:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            return cX, cY
        
        return None, None

    def process_vision(self):
        rs_target_found = False
        
        # 1. Evaluate RealSense Priority
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=50) # Non-blocking short timeout
            color_frame = frames.get_color_frame()
            if color_frame:
                frame_rs = np.asanyarray(color_frame.get_data())
                img_msg = self.bridge.cv2_to_imgmsg(frame_rs, encoding="bgr8")
                self.rs_display_pub.publish(img_msg)
                cX, cY = self.get_red_centroid(frame_rs)

                if cX is not None:
                    rs_target_found = True
                    self.last_realsense_time = self.get_clock().now().nanoseconds
                    
                    if self.active_camera != 'realsense':
                        self.get_logger().info("RealSense regained detection — taking control.")
                        self._reset_pid()
                        self.active_camera = 'realsense'

                    x_robot, y_robot = self._pixel_to_robot_frame(
                        cX, cY, REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y,
                        self.rs_pixels_per_meter, self.rs_robot_x, self.rs_robot_y
                    )
                    dist = math.sqrt(x_robot**2 + y_robot**2)
                    self._run_control(x_robot, y_robot, dist, GOAL_THRESH_REALSENSE)
        except RuntimeError:
            pass # No new frames arrived in time

        # 2. Evaluate Arducam Handoff if RealSense fails
        now_ns = self.get_clock().now().nanoseconds
        realsense_age_sec = (now_ns - self.last_realsense_time) / 1e9

        if not rs_target_found and realsense_age_sec >= REALSENSE_TIMEOUT_SEC:
            if self.cap_arducam.isOpened():
                ret, frame_ard = self.cap_arducam.read()
                
                if ret:
                    # Move publishing inside the check to prevent crashes on dropped frames
                    img_msg = self.bridge.cv2_to_imgmsg(frame_ard, encoding="bgr8")
                    self.arc_display_pub.publish(img_msg)
                    
                    cX, cY = self.get_red_centroid(frame_ard)
                    
                    if cX is not None:
                        if self.active_camera != 'arducam':
                            self.get_logger().info(f"RealSense lost for {realsense_age_sec:.1f}s — Arducam taking control.")
                            self._reset_pid()
                            self.active_camera = 'arducam'

                        x_robot, y_robot = self._pixel_to_robot_frame(
                            cX, cY, ARDUCAM_OFFSET_X, ARDUCAM_OFFSET_Y,
                            self.arc_pixels_per_meter, self.arc_robot_x, self.arc_robot_y
                        )
                        dist = math.sqrt(x_robot**2 + y_robot**2)
                        self._run_control(x_robot, y_robot, dist, GOAL_THRESH_ARDUCAM)
    
    def _run_control(self, x_robot, y_robot, dist, goal_thresh):
        # Goal reached — stop and trigger auger
        if dist <= goal_thresh:
            self.get_logger().info(f"Target reached (dist={dist:.2f}m)! Triggering auger.")
            self._publish_wheels(0.0, 0.0)
            trigger_msg      = String()
            trigger_msg.data = "drill"
            self.auger_trigger_pub.publish(trigger_msg)
            # The node stops publishing here; CommanderNode will route logic or timeout.
            return

        error_theta = math.atan2(y_robot, x_robot)

        self.get_logger().info(
            f"[{self.active_camera}] dist={dist:.2f}m  "
            f"err={np.degrees(error_theta):.1f}deg  "
            f"x_robot={x_robot:.3f} y_robot={y_robot:.3f}",
            throttle_duration_sec=0.5
        )

        # ----------------------------------------------------------
        # PIVOT: full counter-rotation until heading is aligned
        # ----------------------------------------------------------
        if abs(error_theta) > PIVOT_THRESH:
            self._reset_pid()
            if error_theta > 0:
                wl_des = -PIVOT_SPEED
                wr_des =  PIVOT_SPEED
            else:
                wl_des =  PIVOT_SPEED
                wr_des = -PIVOT_SPEED

        # ----------------------------------------------------------
        # DRIVE: heading aligned, PID correction mixed into wheels
        # ----------------------------------------------------------
        else:
            correction = self.pid_steer.update(setpoint=error_theta, output=0.0)
            wl_des = DRIVE_SPEED - correction
            wr_des = DRIVE_SPEED + correction

            max_input = max(abs(wl_des), abs(wr_des))
            if max_input > MAX_ACTUATOR_INPUT:
                scale  = MAX_ACTUATOR_INPUT / max_input
                wl_des *= scale
                wr_des *= scale

        self._publish_wheels(wl_des, wr_des)

    def _publish_wheels(self, left, right):
        msg      = Float32MultiArray()
        msg.data = [float(left), float(right)]
        self.wheel_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GTGControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Gracefully release hardware pipelines
        if hasattr(node, 'pipeline'):
            node.pipeline.stop()
        if hasattr(node, 'cap_arducam') and node.cap_arducam.isOpened():
            node.cap_arducam.release()
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()