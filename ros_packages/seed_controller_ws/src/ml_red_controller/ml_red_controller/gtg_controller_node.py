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
        
        # --- NEW: Matched Path Follower Constants ---
        self.MAX_ACTUATOR_INPUT = 50.0
        self.PIVOT_THRESH = np.deg2rad(15.0)
        self.PIVOT_SPEED = 50.0
        self.PIVOT_BACK_FRACTION = 0.6
        self.DRIVE_SPEED = 50.0
        self.GOAL_THRESH = 0.3 

        # Cooldown Logic
        self.COOLDOWN_DUR = 15.0 
        self.cooldown_end_time = 0

        # PID for steering correction during DRIVE state
        # Ts is kept at 1/30 to match the ~30 FPS of the camera
        self.pid_steer = PID(
            Kp=15.0,
            Ki=0.8,
            Kd=2.0,
            N=15.0,
            Ts=1/30.0,
            umax=self.MAX_ACTUATOR_INPUT,
            umin=-self.MAX_ACTUATOR_INPUT,
            Kaw=2.0
        )

        # Publishers
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/vision/wheel_cmd', 10)
        self.auger_trigger_pub = self.create_publisher(String, '/auger/activate', 10)

        # Subscriber
        self.subscription = self.create_subscription(Point, '/vision/target_point', self.control_callback, 10)

    def load_calibration(self):
        load_file = "calibration_data.npz" 
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
            return  

        cX, cY = msg.x, msg.y

        # 1. Calculate Error Vector (Camera frame to Robot frame)
        dx_px = cX - self.robot_x
        dy_px = self.robot_y - cY 
        
        rel_x = (dx_px / self.pixels_per_meter) - 0.1
        rel_y = (dy_px / self.pixels_per_meter) + 0.16

        # x_robot is forward distance, y_robot is lateral (left/right) distance
        x_robot = rel_y
        y_robot = -rel_x
        dist_to_goal = math.sqrt(x_robot**2 + y_robot**2)

        # 2. Reached the object! Trigger Auger and Enter Cooldown
        if dist_to_goal <= self.GOAL_THRESH:
            self.get_logger().info("Target Reached! Triggering Auger and entering cooldown.")
            
            # Send stop command so wheels don't keep spinning while auger starts
            cmd_msg = Float32MultiArray()
            cmd_msg.data = [0.0, 0.0]
            self.wheel_cmd_pub.publish(cmd_msg)

            # Send trigger to Auger
            trigger_msg = String()
            trigger_msg.data = "drill"
            self.auger_trigger_pub.publish(trigger_msg)
            
            # Start Cooldown
            self.cooldown_end_time = self.get_clock().now().nanoseconds + (self.COOLDOWN_DUR * 1e9)
            return

        # 3. Calculate Desired Heading
        # Since the robot's local frame is always facing 0 radians, 
        # the desired angle relative to the robot is just atan2(y, x).
        error_theta = math.atan2(y_robot, x_robot)

        # ----------------------------------------------------------
        # PIVOT state: bang-bang spin for large heading errors.
        # ----------------------------------------------------------
        if abs(error_theta) > self.PIVOT_THRESH:
            self.pid_steer.istate = 0.0
            self.pid_steer.dstate = 0.0
            self.pid_steer.error_prev = 0.0

            if error_theta > 0:
                # Target to the LEFT — turn left CCW = (-X, +X)
                wl_des = -self.PIVOT_SPEED * self.PIVOT_BACK_FRACTION
                wr_des =  self.PIVOT_SPEED
            else:
                # Target to the RIGHT — turn right CW = (+X, -X)
                wl_des =  self.PIVOT_SPEED
                wr_des = -self.PIVOT_SPEED * self.PIVOT_BACK_FRACTION

        # ----------------------------------------------------------
        # DRIVE state: PID steering correction mixed into both wheels.
        # ----------------------------------------------------------
        else:
            correction = self.pid_steer.update(setpoint=error_theta, output=0.0)

            wl_des = self.DRIVE_SPEED - correction
            wr_des = self.DRIVE_SPEED + correction

        # 4. Scale to fit within actuator limits while preserving the ratio
        max_input = max(abs(wl_des), abs(wr_des))
        if max_input > self.MAX_ACTUATOR_INPUT:
            scale = self.MAX_ACTUATOR_INPUT / max_input
            wl_des *= scale
            wr_des *= scale

        # 5. Publish to the Master Controller
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