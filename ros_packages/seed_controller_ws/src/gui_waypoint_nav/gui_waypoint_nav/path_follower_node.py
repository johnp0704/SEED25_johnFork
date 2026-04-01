import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D, Vector3
from std_msgs.msg import Empty
import numpy as np
import atexit
import time
import os
import json

from gui_waypoint_nav import sabertooth as st 
from gui_waypoint_nav.PID import PID

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Configuration Constants
        self.MAX_ACTUATOR_INPUT = 30
        self.S_MAX = (self.MAX_ACTUATOR_INPUT - 10) * 0.6
        self.ANGLE_THRESH = np.deg2rad(3)
        self.R_wheel = 0.08
        self.K_e = 30
        self.K_theta = -self.K_e * 10
        
        # Dynamic Calibration Variable
        self.L = 0.178 
        self.load_calibration()

        # Initialize PID Controller for Heading
        self.pid_heading = PID(
            Kp=self.K_theta, Ki=0.0, Kd=0.0, Ts=0.1, 
            umax=self.MAX_ACTUATOR_INPUT, umin=-self.MAX_ACTUATOR_INPUT
        )

        # State Variables
        self.robot_pose = None
        self.last_target_time = time.time()
        self.last_override_time = 0.0

        # Initialize Motors
        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            self.get_logger().info("Motors Initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {e}")
        
        atexit.register(self.motor.all_motors_off)

        # Subscribers
        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)
        
        # Calibration Subscribers
        self.create_subscription(Vector3, '/override_motor_cmds', self.override_callback, 10)
        self.create_subscription(Empty, '/calibration_updated', self.reload_callback, 10)

        # Publisher for Dead Reckoning Node
        self.cmd_pub = self.create_publisher(Vector3, '/motor_cmds', 10)
        
        # Safety Timer 
        self.create_timer(0.1, self.safety_check)

    def load_calibration(self):
        calib_file = os.path.expanduser('~/.ros/seed_calibration.json')
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                    self.L = data.get('L', 0.178)
                self.get_logger().info(f"Loaded Kinematics: L = {self.L:.6f}")
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration: {e}")

    def reload_callback(self, msg):
        self.load_calibration()

    def override_callback(self, msg):
        # The calibration node is taking control!
        self.last_override_time = time.time()
        self.motor.updateMotorSpeed(msg.x, msg.y)
        
        # Broadcast the override to the odometry node so the virtual twin tracks it
        cmd_msg = Vector3()
        cmd_msg.x = msg.x
        cmd_msg.y = msg.y
        cmd_msg.z = 0.0
        self.cmd_pub.publish(cmd_msg)

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.last_target_time = time.time()
        
        # Ignore autonomous targets if we are currently running a calibration burst
        if time.time() - self.last_override_time < 0.5:
            return
            
        if self.robot_pose is None:
            return

        x, y, yaw = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
        x_des, y_des = msg.x, msg.y

        dx = x_des - x
        dy = y_des - y
        distance = np.sqrt(dx**2 + dy**2)
        
        S_des = self.K_e * distance
        S_sat = np.clip(S_des, 0.0, self.S_MAX)

        theta_des = np.arctan2(dy, dx)
        error_theta = theta_des - yaw
        error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        w_des = 0.0
        if abs(error_theta_wrapped) > self.ANGLE_THRESH:
            w_des = self.pid_heading.update(setpoint=error_theta_wrapped, output=0.0)

        if abs(error_theta_wrapped) > np.deg2rad(45):
            S_sat = 0.0
        else:
            S_sat = S_sat * np.cos(error_theta_wrapped)

        wr_des = (S_sat - self.L * w_des) / self.R_wheel
        wl_des = (S_sat + self.L * w_des) / self.R_wheel

        maxInput = max(abs(wr_des), abs(wl_des))
        if maxInput > self.MAX_ACTUATOR_INPUT:
            speed_adjust_factor = self.MAX_ACTUATOR_INPUT / maxInput
            wr_des *= speed_adjust_factor
            wl_des *= speed_adjust_factor

        wr_des_sat = np.clip(wr_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)

        self.motor.updateMotorSpeed(wl_des_sat, wr_des_sat)

        cmd_msg = Vector3()
        cmd_msg.x = float(wl_des_sat)
        cmd_msg.y = float(wr_des_sat)
        cmd_msg.z = 0.0
        self.cmd_pub.publish(cmd_msg)

    def safety_check(self):
        # Do not trigger emergency stop if the calibration node is driving
        if time.time() - self.last_override_time < 0.5:
            return
            
        if time.time() - self.last_target_time > 0.5:
            self.motor.updateMotorSpeed(0, 0)
            
            stop_msg = Vector3()
            stop_msg.x = 0.0
            stop_msg.y = 0.0
            stop_msg.z = 0.0
            self.cmd_pub.publish(stop_msg)

    def destroy_node(self):
        self.motor.all_motors_off()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()