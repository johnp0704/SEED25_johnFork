# path_follower_node.py
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

        self.MAX_ACTUATOR_INPUT = 50
        self.S_MAX = 24.0

        self.ANGLE_THRESH = np.deg2rad(2.0)
        self.PIVOT_THRESH = np.deg2rad(40.0)
        self.PIVOT_BACK_FRACTION = 0.3

        self.R_wheel = 0.08
        self.L = 0.178
        self.K_e = 20.0
        self.K_theta = -15.0   # FIXED: flat value, not a multiple of K_e

        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0

        self.motor_disabled = False
        self.vision_wl = 0.0
        self.vision_wr = 0.0
        self.last_vision_time = 0

        self.pid_heading = PID(
            Kp=self.K_theta,
            Ki=0.0,
            Kd=0.0,
            N=15.0,
            Ts=0.1,
            umax=self.MAX_ACTUATOR_INPUT,
            umin=-self.MAX_ACTUATOR_INPUT
        )

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.x_des = None
        self.y_des = None

        self.last_target_time = self.get_clock().now()

        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            self.get_logger().info("Motors initialized. Waiting for commands...")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {e}")

        atexit.register(self.motor.all_motors_off)

        self.create_subscription(Pose2D, '/robot/pose2d', self.pose_callback, 10)
        self.create_subscription(Point, '/robot/current_target', self.target_callback, 10)
        self.create_subscription(Bool, '/motor_disable', self.disable_callback, 10)
        self.create_subscription(Float32MultiArray, '/vision/wheel_cmd', self.vision_cmd_callback, 10)

        self.create_timer(0.1, self.control_loop)

    def disable_callback(self, msg):
        self.motor_disabled = msg.data
        if self.motor_disabled:
            self.get_logger().info("MOTORS DISABLED by End Effector.", throttle_duration_sec=2.0)

    def vision_cmd_callback(self, msg):
        self.vision_wl = msg.data[0]
        self.vision_wr = msg.data[1]
        self.last_vision_time = self.get_clock().now().nanoseconds

    def pose_callback(self, msg):
        yaw = msg.theta
        self.robot_x = msg.x + (self.OFFSET_X * np.cos(yaw) - self.OFFSET_Y * np.sin(yaw))
        self.robot_y = msg.y + (self.OFFSET_X * np.sin(yaw) + self.OFFSET_Y * np.cos(yaw))
        self.robot_yaw = yaw

    def target_callback(self, msg):
        self.last_target_time = self.get_clock().now()
        new_x, new_y = msg.x, msg.y

        if self.x_des is not None:
            if abs(new_x - self.x_des) > 0.05 or abs(new_y - self.y_des) > 0.05:
                self.pid_heading.istate = 0.0
                self.pid_heading.dstate = 0.0
                self.pid_heading.error_prev = 0.0

        self.x_des = new_x
        self.y_des = new_y

    def control_loop(self):
        elapsed_ns = (self.get_clock().now() - self.last_target_time).nanoseconds
        if elapsed_ns > 5e8:
            self.motor.updateMotorSpeed(0, 0)
            return

        if self.motor_disabled:
            self.motor.updateMotorSpeed(0, 0)
            return

        if (self.get_clock().now().nanoseconds - self.last_vision_time) < 5e8:
            self.get_logger().info("Vision GTG Override Active", throttle_duration_sec=1.0)
            self.motor.updateMotorSpeed(self.vision_wl, self.vision_wr)
            return

        if self.robot_x is None or self.x_des is None:
            return

        dx = self.x_des - self.robot_x
        dy = self.y_des - self.robot_y
        dist = np.sqrt(dx**2 + dy**2)

        theta_des = np.arctan2(dy, dx)
        error_theta = np.arctan2(
            np.sin(theta_des - self.robot_yaw),
            np.cos(theta_des - self.robot_yaw)
        )

        w_des = 0.0
        if abs(error_theta) > self.ANGLE_THRESH:
            w_des = self.pid_heading.update(setpoint=error_theta, output=0.0)

        if abs(error_theta) > self.PIVOT_THRESH:
            S_sat = 0.0
        else:
            arc_scale = np.cos(error_theta)
            S_des = self.K_e * dist * arc_scale
            S_sat = np.clip(S_des, 0.0, self.S_MAX)

        wl_des = S_sat + self.L / self.R_wheel * w_des
        wr_des = S_sat - self.L / self.R_wheel * w_des

        if abs(error_theta) > self.PIVOT_THRESH:
            if w_des >= 0:
                pivot_speed = np.clip(abs(w_des), 5.0, self.MAX_ACTUATOR_INPUT)
                wl_des =  pivot_speed
                wr_des = -pivot_speed * self.PIVOT_BACK_FRACTION
            else:
                pivot_speed = np.clip(abs(w_des), 5.0, self.MAX_ACTUATOR_INPUT)
                wl_des = -pivot_speed * self.PIVOT_BACK_FRACTION
                wr_des =  pivot_speed

        max_input = max(abs(wl_des), abs(wr_des))
        if max_input > self.MAX_ACTUATOR_INPUT:
            scale = self.MAX_ACTUATOR_INPUT / max_input
            wl_des *= scale
            wr_des *= scale

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