import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
import numpy as np
import atexit

from gui_waypoint_nav import sabertooth as st 
from gui_waypoint_nav.PID import PID
from geometry_msgs.msg import Vector3

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Configuration Constants
        self.MAX_ACTUATOR_INPUT = 30
        self.S_MAX = (self.MAX_ACTUATOR_INPUT - 10) * 0.6
        self.ANGLE_THRESH = np.deg2rad(3)
        self.R_wheel = 0.08
        self.L = 0.178
        self.K_e = 30
        self.K_theta = -self.K_e * 10

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

        # Publisher
        self.cmd_pub = self.create_publisher(Vector3, '/motor_cmds', 10)
        # Safety Timer 
        self.create_timer(0.1, self.safety_check)

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        
        if self.robot_pose is None:
            return

        # Unpack state
        x, y, yaw = self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
        x_des, y_des = msg.x, msg.y

        # 1. Calculate Velocity/Translation Command (P-Control)
        Ux_des = self.K_e * (x_des - x)
        Uy_des = self.K_e * (y_des - y)
        S_des = np.sqrt(Ux_des**2 + Uy_des**2)
        S_sat = np.clip(S_des, -self.S_MAX, self.S_MAX)

        # 2. Calculate Angular Command (PID Control)
        theta_des = np.arctan2(Uy_des, Ux_des)
        error_theta = theta_des - yaw
        
        # Wrap angle to [-pi, pi]
        error_theta_wrapped = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        w_des = 0
        if abs(error_theta_wrapped) > self.ANGLE_THRESH:
            # Trick: Pass the wrapped error as the setpoint, and 0 as the output
            w_des = self.pid_heading.update(setpoint=error_theta_wrapped, output=0.0)

        # 3. Kinematics (Convert to left/right wheel speeds)
        wr_des = (S_sat - self.L * w_des) / self.R_wheel
        wl_des = (S_sat + self.L * w_des) / self.R_wheel

        # 4. Saturation and Scaling
        maxInput = max(abs(wr_des), abs(wl_des))
        if maxInput > self.MAX_ACTUATOR_INPUT:
            speed_adjust_factor = self.MAX_ACTUATOR_INPUT / maxInput
            wr_des *= speed_adjust_factor
            wl_des *= speed_adjust_factor

        wr_des_sat = np.clip(wr_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)
        wl_des_sat = np.clip(wl_des, -self.MAX_ACTUATOR_INPUT, self.MAX_ACTUATOR_INPUT)

        # Send commands to motors
        self.motor.updateMotorSpeed(wl_des_sat, wr_des_sat)

        # Broadcast the commanded speeds to the dead-reckoning node
        cmd_msg = Vector3()
        cmd_msg.x = float(wl_des_sat) # Left wheel speed
        cmd_msg.y = float(wr_des_sat) # Right wheel speed
        cmd_msg.z = 0.0
        self.cmd_pub.publish(cmd_msg)

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