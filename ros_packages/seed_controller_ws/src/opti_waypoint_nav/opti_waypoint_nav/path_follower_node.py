import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
import numpy as np
import atexit

# Import your custom modules from the package
from opti_waypoint_nav import sabertooth as st 
from opti_waypoint_nav.PID import PID

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Configuration Constants
        self.MAX_ACTUATOR_INPUT = 30
        self.S_MAX = (self.MAX_ACTUATOR_INPUT - 10) * 0.6
        self.ANGLE_THRESH = np.deg2rad(1)   
        self.PIVOT_THRESH = np.deg2rad(3)  
        
        self.R_wheel = 0.08
        self.L = 0.178
        self.K_e = 30
        self.K_theta = -self.K_e * 10

        # --- NEW: Center Tuning Constants ---
        # Offset from the OptiTrack rigid body origin to the true robot control center (in meters).
        # OFFSET_X: Positive is forward, negative is backward.
        # OFFSET_Y: Positive is left, negative is right.
        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0  
        
        # --- NEW: Pause State Variables ---
        self.PAUSE_DURATION = 3.0  # seconds
        self.current_target_coords = None
        self.pause_end_time = None

        # Initialize PID Controller for Heading
        self.pid_heading = PID(
            Kp=self.K_theta, 
            Ki=0.0, 
            Kd=0.0, 
            Ts=0.1, 
            umax=self.MAX_ACTUATOR_INPUT, 
            umin=-self.MAX_ACTUATOR_INPUT
        )

        # State Variables
        self.robot_pose = None
        self.last_target_time = self.get_clock().now()

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

        # Safety Timer 
        self.create_timer(0.1, self.safety_check)

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        
        if self.robot_pose is None:
            return

        # 1. Unpack raw state and target
        raw_x, raw_y, yaw = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
        x_des, y_des = msg.x, msg.y

        # --- 2. Center Tuning ---
        # Apply the rotation matrix to shift the control center
        x = raw_x + (self.OFFSET_X * np.cos(yaw) - self.OFFSET_Y * np.sin(yaw))
        y = raw_y + (self.OFFSET_X * np.sin(yaw) + self.OFFSET_Y * np.cos(yaw))

        # --- 3. Waypoint Pause Logic ---
        # Detect if the target has changed to a new waypoint coordinate
        if self.current_target_coords is None or (self.current_target_coords[0] != x_des or self.current_target_coords[1] != y_des):
            self.current_target_coords = (x_des, y_des)
            self.get_logger().info(f"New target acquired. Pausing for {self.PAUSE_DURATION} seconds.")
            # Calculate the future time when the pause should end (converted to nanoseconds)
            self.pause_end_time = self.get_clock().now().nanoseconds + (self.PAUSE_DURATION * 1e9)

        # If we are currently inside the pause window, hold motors at 0 and skip control math
        if self.pause_end_time and self.get_clock().now().nanoseconds < self.pause_end_time:
            self.motor.updateMotorSpeed(0, 0)
            return

        # 4. Calculate pure distances and desired angle
        dx = x_des - x
        dy = y_des - y
        
        theta_des = np.arctan2(dy, dx)
        error_theta = theta_des - yaw
        
        # Wrap angle to [-pi, pi]
        error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        # 5. Calculate Angular Command (PID Control)
        w_des = 0
        if abs(error_theta_wrapped) > self.ANGLE_THRESH:
            w_des = self.pid_heading.update(setpoint=error_theta_wrapped, output=0.0)

        # 6. Calculate Velocity/Translation Command (Pivot-Then-Go Logic)
        if abs(error_theta_wrapped) > self.PIVOT_THRESH:
            # If we are facing the wrong way, do not move forward. Just pivot.
            S_sat = 0.0
        else:
            # If heading is good, calculate forward speed based on distance
            S_des = self.K_e * np.sqrt(dx**2 + dy**2)
            S_sat = np.clip(S_des, -self.S_MAX, self.S_MAX)

        # 7. Kinematics (Convert to left/right wheel speeds)
        wr_des = (S_sat - self.L * w_des) / self.R_wheel
        wl_des = (S_sat + self.L * w_des) / self.R_wheel

        # 8. Saturation and Scaling
        maxInput = max(abs(wr_des), abs(wl_des))
        if maxInput > self.MAX_ACTUATOR_INPUT:
            speed_adjust_factor = self.MAX_ACTUATOR_INPUT / maxInput
            wr_des *= speed_adjust_factor
            wl_des *= speed_adjust_factor

        wr_des_sat = np.clip(wr_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)

        # Send commands to motors
        self.motor.updateMotorSpeed(wl_des_sat, wr_des_sat)

    def safety_check(self):
        # Stop motors if no target seen in 0.5 seconds (e.g., path finished or tracking lost)
        if (self.get_clock().now() - self.last_target_time).nanoseconds > 5e8:
            self.motor.updateMotorSpeed(0, 0)

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