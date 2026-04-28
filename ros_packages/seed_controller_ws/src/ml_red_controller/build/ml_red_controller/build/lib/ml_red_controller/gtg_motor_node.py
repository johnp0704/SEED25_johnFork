import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import atexit

from ml_red_controller.sabertooth import SaberToothMotorDriver


class GTGMotorNode(Node):
    """
    Standalone Sabertooth driver for the vision GTG system.
    Runs its own 20Hz timer to drive the trapezoidal ramp smoothly.
    Stops if no command received for 500ms, or if auger is active.
    """
    def __init__(self):
        super().__init__('gtg_motor_node')

        self.motor_disabled = False
        self.last_cmd_time  = self.get_clock().now()
        self._cmd_left  = 0.0
        self._cmd_right = 0.0

        try:
            self.motor = SaberToothMotorDriver(
                motor1_reversed=True,
                motor2_reversed=True,
                accel_rate=80.0,
                decel_rate=120.0,
                call_rate_hz=20.0
            )
            self.get_logger().info("GTG Motor Node initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Sabertooth: {e}")

        atexit.register(self.motor.all_motors_off)

        self.create_subscription(Float32MultiArray, '/vision/wheel_cmd', self.cmd_callback, 10)
        self.create_subscription(Bool, '/motor_disable', self.disable_callback, 10)

        # 20Hz motor loop — must match call_rate_hz above
        self.create_timer(0.05, self.motor_loop)

    def disable_callback(self, msg):
        self.motor_disabled = msg.data
        if self.motor_disabled:
            self.get_logger().info("MOTORS DISABLED by auger.", throttle_duration_sec=2.0)

    def cmd_callback(self, msg):
        self.last_cmd_time  = self.get_clock().now()
        self._cmd_left  = msg.data[0]
        self._cmd_right = msg.data[1]

    def motor_loop(self):
        if self.motor_disabled:
            self.motor.updateMotorSpeed(0, 0)
            return

        elapsed_ns = (self.get_clock().now() - self.last_cmd_time).nanoseconds
        if elapsed_ns > 5e8:
            self.motor.updateMotorSpeed(0, 0)
            return

        self.motor.updateMotorSpeed(self._cmd_left, self._cmd_right)

    def destroy_node(self):
        self.motor.all_motors_off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GTGMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()