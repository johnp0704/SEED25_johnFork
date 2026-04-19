import rclpy
from rclpy.node import Node
import time

# Import your custom classes from the package module
from motor_tester.sabertooth import SaberToothMotorDriver
from motor_tester.PID import PID  # Included for future closed-loop integration

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        self.get_logger().info('Initializing Motor Test Node...')
        
        # Initialize the driver. Change to True if your wiring is backwards
        try:
            self.driver = SaberToothMotorDriver(motor1_reversed=False, motor2_reversed=False)
            self.get_logger().info('Sabertooth Serial Port Opened Successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Sabertooth driver: {e}')
            return

        # Execute the test sequence
        self.run_tests()
        
    def run_tests(self):
        self.get_logger().info('Starting Test Sequence in 3 seconds...')
        time.sleep(3)

        # Helper function to keep the test code clean
        def execute_test(test_name, left_speed, right_speed, duration=1.5):
            self.get_logger().info(f'Running: {test_name} | L: {left_speed}% | R: {right_speed}%')
            self.driver.updateMotorSpeed(left_speed, right_speed)
            time.sleep(duration)
            self.driver.all_motors_off()
            self.get_logger().info('Motors Stopped.')
            time.sleep(1.5) # Pause between tests for safety

        # 1. Pivots (Wheels moving opposite directions)
        execute_test("Pivot Right", -100, 100)
        execute_test("Pivot Left", 100, -100)

        # 2. Straight lines
        execute_test("Backward", 100, 100)
        execute_test("Forward", -100, -100)

        # 3. Arcing turns Forward (One wheel faster than the other)
        execute_test("Arc Turn Backward Left", 50, 100)
        execute_test("Arc Turn Backward Right", 100, 50)

        # 4. Arcing turns Backward
        execute_test("Arc Turn Forward Left", -50, -100)
        execute_test("Arc Turn Forward Right", -100, -50)

        self.get_logger().info('All motor tests completed successfully. Shutting down.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    
    # We do not use rclpy.spin(node) here because the tests run synchronously in __init__
    # Once run_tests() finishes, we can just clean up and exit.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()