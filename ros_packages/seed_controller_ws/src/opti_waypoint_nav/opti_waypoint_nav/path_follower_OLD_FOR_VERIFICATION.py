import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import atexit

from opti_waypoint_nav import sabertooth as st 
from opti_waypoint_nav.PID import PID

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.MAX_ACTUATOR_INPUT = 60
        self.S_MAX = (self.MAX_ACTUATOR_INPUT - 10) * 0.6
        self.ANGLE_THRESH = np.deg2rad(1)   
        self.PIVOT_THRESH = np.deg2rad(3)  
        self.R_wheel = 0.08
        self.L = 0.178
        self.K_e = 50
        self.K_theta = -self.K_e * 10

        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0  
        
        self.PAUSE_DURATION = 3.0
        self.current_target_coords = None
        self.pause_end_time = None

        # --- NEW: Priority State Tracking ---
        self.motor_disabled = False
        self.vision_active = False
        self.vision_wl = 0.0
        self.vision_wr = 0.0
        self.last_vision_time = 0

        self.pid_heading = PID(Kp=self.K_theta, Ki=0.0, Kd=0.0, Ts=0.1, umax=self.MAX_ACTUATOR_INPUT, umin=-self.MAX_ACTUATOR_INPUT)
        self.robot_pose = None
        self.last_target_time = self.get_clock().now()

        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            self.get_logger().info("Motors Initialized. Waiting for commands...")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {e}")
        
        atexit.register(self.motor.all_motors_off)

        # Subscribers
        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)
        
        # --- NEW: Priority Subscribers ---
        self.create_subscription(Bool, '/motor_disable', self.disable_callback, 10)
        self.create_subscription(Float32MultiArray, '/vision/wheel_cmd', self.vision_cmd_callback, 10)

        self.create_timer(0.1, self.control_loop)

    def disable_callback(self, msg):
        self.motor_disabled = msg.data
        if self.motor_disabled:
            self.get_logger().info("MOTORS DISABLED by End Effector.", throttle_duration_sec=2.0)

    def vision_cmd_callback(self, msg):
        """Receives wheel speeds from the GTG node."""
        self.vision_wl = msg.data[0]
        self.vision_wr = msg.data[1]
        self.last_vision_time = self.get_clock().now().nanoseconds

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        
        if self.robot_pose is None:
            return

        # 1. Unpack raw state and target
        raw_x, raw_y, yaw = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
        x_des, y_des = msg.x, msg.y

        # 2. Center Tuning
        self.x = raw_x + (self.OFFSET_X * np.cos(yaw) - self.OFFSET_Y * np.sin(yaw))
        self.y = raw_y + (self.OFFSET_X * np.sin(yaw) + self.OFFSET_Y * np.cos(yaw))
        self.yaw = yaw

        # 3. Waypoint Pause Logic
        if self.current_target_coords is None or (self.current_target_coords[0] != x_des or self.current_target_coords[1] != y_des):
            self.current_target_coords = (x_des, y_des)
            self.get_logger().info(f"New target acquired. Pausing for {self.PAUSE_DURATION} seconds.")
            self.pause_end_time = self.get_clock().now().nanoseconds + (self.PAUSE_DURATION * 1e9)
            
        self.x_des = x_des
        self.y_des = y_des

    def control_loop(self):
        """Central loop handling priorities: 1. Disabled, 2. Vision, 3. Waypoint"""
        # Safety Check
        if (self.get_clock().now() - self.last_target_time).nanoseconds > 5e8:
            self.motor.updateMotorSpeed(0, 0)
            return

        # PRIORITY 1: Auger is active, absolute zero.
        if self.motor_disabled:
            self.motor.updateMotorSpeed(0, 0)
            return

        # PRIORITY 2: Vision GTG is active (message received in the last 0.5 seconds)
        if (self.get_clock().now().nanoseconds - self.last_vision_time) < 5e8:
            self.get_logger().info("Vision GTG Override Active", throttle_duration_sec=1.0)
            self.motor.updateMotorSpeed(self.vision_wl, self.vision_wr)
            return

        # PRIORITY 3: Waypoint Navigation (Default)
        if self.pause_end_time and self.get_clock().now().nanoseconds < self.pause_end_time:
            self.motor.updateMotorSpeed(0, 0)
            return

        if not hasattr(self, 'x_des'):
            return

        # --- Standard Waypoint Math ---
        dx = self.x_des - self.x
        dy = self.y_des - self.y
        
        theta_des = np.arctan2(dy, dx)
        error_theta_wrapped = np.arctan2(np.sin(theta_des - self.yaw), np.cos(theta_des - self.yaw))

        w_des = 0
        if abs(error_theta_wrapped) > self.ANGLE_THRESH:
            w_des = self.pid_heading.update(setpoint=error_theta_wrapped, output=0.0)

        if abs(error_theta_wrapped) > self.PIVOT_THRESH:
            S_sat = 0.0
        else:
            S_des = self.K_e * np.sqrt(dx**2 + dy**2)
            S_sat = np.clip(S_des, -self.S_MAX, self.S_MAX)

        wr_des = (S_sat - self.L * w_des) / self.R_wheel
        wl_des = (S_sat + self.L * w_des) / self.R_wheel

        maxInput = max(abs(wr_des), abs(wl_des))
        if maxInput > self.MAX_ACTUATOR_INPUT:
            scale = self.MAX_ACTUATOR_INPUT / maxInput
            wr_des *= scale
            wl_des *= scale

        self.motor.updateMotorSpeed(
            np.clip(wl_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT), 
            np.clip(wr_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)
        )

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