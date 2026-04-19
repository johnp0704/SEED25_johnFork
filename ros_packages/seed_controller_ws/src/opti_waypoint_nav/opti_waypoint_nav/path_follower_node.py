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

        # Above this, bang-bang pivot before driving
        self.PIVOT_THRESH = np.deg2rad(15.0)

        # Fixed speeds for bang-bang pivot
        self.PIVOT_SPEED = 35.0
        self.PIVOT_BACK_FRACTION = 0.4

        # Forward drive speed — constant, no distance slowdown
        self.DRIVE_SPEED = 35.0

        # Verified convention (True, True):
        #   Forward    = (+X, +X)
        #   Turn left  = (-X, +X)  CCW top-down
        #   Turn right = (+X, -X)  CW top-down

        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0

        self.motor_disabled = False
        self.vision_wl = 0.0
        self.vision_wr = 0.0
        self.last_vision_time = 0

        # PID for steering correction during DRIVE state.
        # Output is a signed correction value added/subtracted to wheel speeds.
        # Kp=15: at 15deg error (0.26 rad) -> correction = 15 * 0.26 = 3.9 units
        # Start with P only, add Kd once P is tuned, Ki last.
        # N is the derivative filter coefficient — 10-20 is a good range.
        # Kaw is anti-windup gain — set to 1.0 to start.
        self.pid_steer = PID(
            Kp=15.0,
            Ki=0.0,
            Kd=2.0,
            N=15.0,
            Ts=0.1,
            umax=self.MAX_ACTUATOR_INPUT,
            umin=-self.MAX_ACTUATOR_INPUT,
            Kaw=1.0
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
            self.get_logger().info("MOTORS DISABLED.", throttle_duration_sec=2.0)

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

        # Reset PID when waypoint changes to avoid stale integral/derivative
        if self.x_des is not None:
            if abs(new_x - self.x_des) > 0.05 or abs(new_y - self.y_des) > 0.05:
                self.pid_steer.istate = 0.0
                self.pid_steer.dstate = 0.0
                self.pid_steer.error_prev = 0.0

        self.x_des = new_x
        self.y_des = new_y

    def control_loop(self):
        # --- Safety: no recent target ---
        elapsed_ns = (self.get_clock().now() - self.last_target_time).nanoseconds
        if elapsed_ns > 5e8:
            self.motor.updateMotorSpeed(0, 0)
            return

        # --- Priority 1: Auger active ---
        if self.motor_disabled:
            self.motor.updateMotorSpeed(0, 0)
            return

        # --- Priority 2: Vision override ---
        if (self.get_clock().now().nanoseconds - self.last_vision_time) < 5e8:
            self.get_logger().info("Vision GTG Override Active", throttle_duration_sec=1.0)
            self.motor.updateMotorSpeed(self.vision_wl, self.vision_wr)
            return

        # --- Priority 3: Waypoint nav ---
        if self.robot_x is None or self.x_des is None:
            return

        dx = self.x_des - self.robot_x
        dy = self.y_des - self.robot_y

        theta_des = np.arctan2(dy, dx)
        error_theta = np.arctan2(
            np.sin(theta_des - self.robot_yaw),
            np.cos(theta_des - self.robot_yaw)
        )

        self.get_logger().info(
            f"error={np.degrees(error_theta):.1f}deg  "
            f"robot=({self.robot_x:.2f},{self.robot_y:.2f})  "
            f"target=({self.x_des:.2f},{self.y_des:.2f})",
            throttle_duration_sec=0.5
        )

        # ----------------------------------------------------------
        # PIVOT state: bang-bang spin for large heading errors.
        # Bypass PID entirely — it can't overcome stiction at small outputs.
        # Also reset PID state so it starts fresh when we enter DRIVE.
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

            self.motor.updateMotorSpeed(wl_des, wr_des)
            return

        # ----------------------------------------------------------
        # DRIVE state: PID steering correction mixed into both wheels.
        # correction > 0 means turn left: slow left, speed right
        # correction < 0 means turn right: speed left, slow right
        #
        # By subtracting correction from left and adding to right,
        # a large enough correction will push one wheel negative,
        # giving a sharp counter-rotating arc without a full stop.
        # ----------------------------------------------------------
        correction = self.pid_steer.update(setpoint=error_theta, output=0.0)

        wl_des = self.DRIVE_SPEED - correction
        wr_des = self.DRIVE_SPEED + correction

        # Scale to fit within actuator limits while preserving the ratio
        max_input = max(abs(wl_des), abs(wr_des))
        if max_input > self.MAX_ACTUATOR_INPUT:
            scale = self.MAX_ACTUATOR_INPUT / max_input
            wl_des *= scale
            wr_des *= scale

        self.motor.updateMotorSpeed(wl_des, wr_des)

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