import rclpy
from rclpy.node import Node
import time

from motor_tester.sabertooth import SaberToothMotorDriver
from motor_tester.PID import PID

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        self.get_logger().info('Initializing Motor Test Node...')
        
        try:
            self.driver = SaberToothMotorDriver(motor1_reversed=False, motor2_reversed=False)
            self.get_logger().info('Sabertooth Serial Port Opened Successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Sabertooth driver: {e}')
            return

        self.run_tests()
        
    def execute_test(self, test_name, left_speed, right_speed, duration=1.5):
        self.get_logger().info(f'Running: {test_name} | L: {left_speed}% | R: {right_speed}%')
        
        dt = 0.1
        elapsed = 0.0
        while elapsed < duration:
            self.driver.updateMotorSpeed(left_speed, right_speed)
            time.sleep(dt)
            elapsed += dt
        
        self.driver.all_motors_off()
        self.get_logger().info('Motors Stopped.')
        time.sleep(1.5)

    def run_tests(self):
        self.get_logger().info('Starting Test Sequence in 3 seconds...')
        time.sleep(3)

        # 1. Pivots
        self.execute_test("Pivot Right", -50, 50)
        self.execute_test("Pivot Left", 50, -50)

        # 2. Straight lines
        self.execute_test("Backward", 50, 50)
        self.execute_test("Forward", -50, -50)

        # 3. Arcing turns Backward
        self.execute_test("Arc Turn Backward Left", 25, 75)
        self.execute_test("Arc Turn Backward Right", 75, 25)

        # 4. Arcing turns Forward
        self.execute_test("Arc Turn Forward Left", -25, -75)
        self.execute_test("Arc Turn Forward Right", -75, -25)

        self.get_logger().info('All motor tests completed successfully. Shutting down.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()