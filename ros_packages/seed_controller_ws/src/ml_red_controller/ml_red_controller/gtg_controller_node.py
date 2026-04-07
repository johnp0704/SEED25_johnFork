import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import math
import os
from ml_red_controller.PID import PID 

class GTGControllerNode(Node):
    def __init__(self):
        super().__init__('gtg_controller_node')
        
        self.load_calibration()
        self.MAX_ACTUATOR_INPUT = 30 
        self.S_MAX = (self.MAX_ACTUATOR_INPUT - 10) * 0.6
        self.GOAL_THRESH = 0.3 
        self.ANGLE_THRESH = np.deg2rad(3) 
        self.PIVOT_THRESHOLD = np.deg2rad(15)
        self.R_wheel = 0.08 
        self.L = 0.178 
        self.K_e = 30
        self.K_theta = -self.K_e * 10 

        # --- NEW: Cooldown Logic ---
        # After triggering the auger, ignore vision for this many seconds to move away
        self.COOLDOWN_DUR = 15.0 
        self.cooldown_end_time = 0

        self.pid_heading = PID(Kp=self.K_theta, Ki=0.0, Kd=0.0, Ts=1/30, umin=-200, umax=200)

        # Publishers
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/vision/wheel_cmd', 10)
        self.auger_trigger_pub = self.create_publisher(String, '/auger/activate', 10)

        # Subscriber
        self.subscription = self.create_subscription(Point, '/vision/target_point', self.control_callback, 10)

    def load_calibration(self):
        load_file = "calibration_data.npz" # Assuming this is in the execution directory
        if os.path.exists(load_file):
            data = np.load(load_file)
            self.pixels_per_meter = float(data['pixels_per_meter'])
            self.robot_x = int(data['robot_x'])
            self.robot_y = int(data['robot_y'])
        else:
            self.get_logger().error("Calibration file not found!")

    def control_callback(self, msg):
        # 0. Check Cooldown
        if self.get_clock().now().nanoseconds < self.cooldown_end_time:
            return  # Ignoring vision while in cooldown

        cX, cY = msg.x, msg.y

        # 1. Calculate Error Vector
        dx_px = cX - self.robot_x
        dy_px = self.robot_y - cY 
        
        rel_x = (dx_px / self.pixels_per_meter) - 0.1
        rel_y = (dy_px / self.pixels_per_meter) + 0.16

        x_robot = rel_y
        y_robot = -rel_x
        dist_to_goal = math.sqrt(x_robot**2 + y_robot**2)

        # 2. Reached the object! Trigger Auger and Enter Cooldown
        if dist_to_goal <= self.GOAL_THRESH:
            self.get_logger().info("Target Reached! Triggering Auger and entering cooldown.")
            
            # Send trigger to Auger
            trigger_msg = String()
            trigger_msg.data = "drill"
            self.auger_trigger_pub.publish(trigger_msg)
            
            # Start Cooldown
            self.cooldown_end_time = self.get_clock().now().nanoseconds + (self.COOLDOWN_DUR * 1e9)
            return

        # 3. Calculate Desired Heading
        theta_des = math.atan2(self.K_e * y_robot, self.K_e * x_robot)
        theta_des_wrapped = math.atan2(math.sin(theta_des), math.cos(theta_des))

        # 4. Pivot vs Drive
        if abs(theta_des_wrapped) > self.PIVOT_THRESHOLD:
            S_sat = 0
        else:
            Ux_des = self.K_e * x_robot
            Uy_des = self.K_e * y_robot
            S_sat = np.clip(math.sqrt(Ux_des**2 + Uy_des**2), -self.S_MAX, self.S_MAX)

        # 5. Angular PID Update
        w_des = self.pid_heading.update(theta_des_wrapped, 0)
        if abs(theta_des_wrapped) < self.ANGLE_THRESH:
            w_des = 0

        # 6. Kinematics
        wr_des = (S_sat - self.L * w_des) / self.R_wheel
        wl_des = (S_sat + self.L * w_des) / self.R_wheel

        maxInput = max(abs(wr_des), abs(wl_des))
        if maxInput > self.MAX_ACTUATOR_INPUT:
            scale = self.MAX_ACTUATOR_INPUT / maxInput
            wr_des *= scale
            wl_des *= scale

        # 7. Publish to the Master Controller instead of hardware
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(wl_des), float(wr_des)]
        self.wheel_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GTGControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()