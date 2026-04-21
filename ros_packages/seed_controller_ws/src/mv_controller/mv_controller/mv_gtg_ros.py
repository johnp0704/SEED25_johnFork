'''
John Poirier
UVM - Senior Experience in Engineering Design
The Autonomous Weeder Robot
'''

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import atexit
import math

# === Custom Imports ===
from ament_index_python.packages import get_package_share_directory
import pkg_resources
from mv_controller.RedLiveDetection import transform_to_bev, detect_red_center
import mv_controller.sabertooth as st
from mv_controller.PID import PID 

class ControllerNode(Node):
    def __init__(self):
        print("Starting MV GTG Controller Node")
        super().__init__('robot_controller_node')
        
        # === 1) Parameters & Constants ===
        import pkg_resources
        default_calib_path = pkg_resources.resource_filename('mv_controller', 'calibration_data.npz')
        
        self.declare_parameter('calibration_file', default_calib_path)
        load_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        
        self.max_actuator_input = 30.0
        self.s_max = (self.max_actuator_input - 10) * 0.6
        self.goal_thresh = 0.3
        self.angle_thresh = np.deg2rad(3)
        self.pivot_threshold = np.deg2rad(15)
        self.r_wheel = 0.08
        self.l_dist = 0.178
        self.k_e = 30
        self.k_theta = -self.k_e * 10

        # === 2) Load Calibration ===
        if not os.path.exists(load_file):
            self.get_logger().error(f"Calibration file not found at {load_file}")
            raise FileNotFoundError()

        data = np.load(load_file)
        self.matrix = data['matrix']
        self.px_per_m = float(data['pixels_per_meter'])
        self.bev_w = int(data['bev_width'])
        self.bev_h = int(data['bev_height'])
        self.robot_x = int(data['robot_x'])
        self.robot_y = int(data['robot_y'])

        # === 3) Hardware Initialization ===
        self.init_camera()
        self.init_motors()

        # === 4) PID Controller ===
        self.pid_heading = PID(
            Kp=self.k_theta, Ki=0.0, Kd=0.0, 
            Ts=1/30, umin=-200, umax=200
        )

        # === 5) ROS2 Timer (30Hz) ===
        self.timer = self.create_timer(1/30, self.control_loop)
        self.get_logger().info("Robot Controller Node Started (Pivot-First Mode)")

    def init_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def init_motors(self):
        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            atexit.register(self.motor.all_motors_off)
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize motors: {e}")
            raise e

    def control_loop(self):
        # === Vision Step ===
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        bev_image = transform_to_bev(frame, self.matrix, self.bev_w, self.bev_h)
        target_center = detect_red_center(bev_image)

        # Robot reference point
        cv2.circle(bev_image, (self.robot_x, self.robot_y), 10, (255, 0, 0), -1)

        # === Control Logic ===
        if target_center:
            cX, cY = target_center
            cv2.circle(bev_image, (cX, cY), 7, (0, 255, 0), -1)

            # Error Calculation
            dx_px = cX - self.robot_x
            dy_px = self.robot_y - cY 
            
            rel_x = (dx_px / self.px_per_m) - 1  # Tuning offsets
            rel_y = (dy_px / self.px_per_m) + 0.16

            x_robot = rel_y
            y_robot = -rel_x
            dist_to_goal = math.sqrt(x_robot**2 + y_robot**2)

            if dist_to_goal > self.goal_thresh:
                theta_des = math.atan2(self.k_e * y_robot, self.k_e * x_robot)
                theta_err = math.atan2(math.sin(theta_des), math.cos(theta_des))

                # Mode Selection
                if abs(theta_err) > self.pivot_threshold:
                    mode, s_sat = "PIVOT", 0.0
                else:
                    mode, s_sat = "DRIVE", np.clip(math.sqrt((self.k_e*x_robot)**2 + (self.k_e*y_robot)**2), -self.s_max, self.s_max)

                # Angular PID
                w_des = self.pid_heading.update(theta_err, 0)
                if abs(theta_err) < self.angle_thresh:
                    w_des = 0.0

                # Kinematics
                wr = (s_sat - self.l_dist * w_des) / self.r_wheel
                wl = (s_sat + self.l_dist * w_des) / self.r_wheel

                # Saturation
                max_in = max(abs(wr), abs(wl))
                if max_in > self.max_actuator_input:
                    scale = self.max_actuator_input / max_in
                    wr, wl = wr * scale, wl * scale

                self.motor.updateMotorSpeed(wl, wr)
                
                # Feedback
                label = f"[{mode}] Err:{np.rad2deg(theta_err):.0f}deg"
                cv2.putText(bev_image, label, (cX, cY - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                self.get_logger().info("Goal Reached", once=True)
                self.motor.updateMotorSpeed(0, 0)
        else:
            self.motor.updateMotorSpeed(0, 0)

        # UI
        cv2.imshow("Robot Controller View", bev_image)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

    def stop(self):
        self.motor.all_motors_off()
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
