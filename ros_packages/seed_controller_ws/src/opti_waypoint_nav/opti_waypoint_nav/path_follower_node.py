import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import atexit

from opti_waypoint_nav import sabertooth as st

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        self.MAX_ACTUATOR_INPUT = 50
        self.PIVOT_THRESH = np.deg2rad(15.0)
        self.PIVOT_SPEED = 35.0
        self.PIVOT_BACK_FRACTION = 0.4
        self.DRIVE_SPEED = 35.0
        self.K_steer = 25.0

        self.R_wheel = 0.08
        self.L = 0.178
        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0

        self.motor_disabled = False
        self.vision_wl = 0.0
        self.vision_wr = 0.0
        self.last_vision_time = 0

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.x_des = None
        self.y_des = None

        self.last_target_time = self.get_clock().now()

        try:
            # False, False — matches verified motor test convention
            self.motor = st.SaberToothMotorDriver(False, False)
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
        self.x_des = msg.x
        self.y_des = msg.y

    def control_loop(self):

        self.motor.updateMotorSpeed(35, 35); return
        self.motor.updateMotorSpeed(-35, 35); return
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
        # PIVOT: spin in place to align heading
        # Verified against motor test (False, False):
        #   Turn left  = updateMotorSpeed(+50, -50)
        #   Turn right = updateMotorSpeed(-50, +50)
        # error_theta > 0 means target is to the LEFT
        # error_theta < 0 means target is to the RIGHT
        # ----------------------------------------------------------
        if abs(error_theta) > self.PIVOT_THRESH:
            if error_theta > 0:
                # Turn left
                wl_des =  self.PIVOT_SPEED
                wr_des = -self.PIVOT_SPEED * self.PIVOT_BACK_FRACTION
            else:
                # Turn right
                wl_des = -self.PIVOT_SPEED * self.PIVOT_BACK_FRACTION
                wr_des =  self.PIVOT_SPEED

            self.motor.updateMotorSpeed(wl_des, wr_des)
            return

        # ----------------------------------------------------------
        # DRIVE: go forward with proportional steering correction
        # Verified against motor test (False, False):
        #   Forward = updateMotorSpeed(-X, -X)
        # error_theta > 0 (target left): need to arc left
        #   → left wheel slower (less negative), right wheel faster (more negative)
        # error_theta < 0 (target right): need to arc right
        #   → right wheel slower, left wheel faster
        # ----------------------------------------------------------
        correction = self.K_steer * error_theta

        # Both negative for forward; correction steers by making one
        # wheel less negative (slower) and the other more negative (faster)
        wl_des = -self.DRIVE_SPEED + correction
        wr_des = -self.DRIVE_SPEED - correction

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