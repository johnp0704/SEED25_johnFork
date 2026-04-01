import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
import json
import os
import time
import threading
import numpy as np

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.get_logger().info("Kinematic Calibrator Ready. Waiting for GUI trigger...")

        self.create_subscription(Empty, '/start_calibration', self.trigger_calib, 10)
        self.override_pub = self.create_publisher(Vector3, '/override_motor_cmds', 10)
        self.update_pub = self.create_publisher(Empty, '/calibration_updated', 10)
        
        self.calib_file = os.path.expanduser('~/.ros/seed_calibration.json')
        self.is_calibrating = False

    def trigger_calib(self, msg):
        if self.is_calibrating:
            self.get_logger().warn("Calibration already in progress! Check your terminal.")
            return
            
        self.is_calibrating = True
        # Run in a separate thread so it doesn't block the ROS spinner
        threading.Thread(target=self.run_calibration, daemon=True).start()

    def fire_burst(self, left_speed, right_speed, duration):
        # Continuously publish so the path follower's safety timer doesn't interrupt it
        start_time = time.time()
        while time.time() - start_time < duration:
            self.send_override(left_speed, right_speed)
            time.sleep(0.1)
        
        # Send the stop command a few times to ensure it arrives
        for _ in range(3):
            self.send_override(0.0, 0.0)
            time.sleep(0.1)

    def run_calibration(self):
        TEST_SPEED = 20.0
        TEST_DURATION = 3.0

        print("\n" + "="*50)
        print(" DIFFERENTIAL DRIVE KINEMATIC CALIBRATOR")
        print("="*50)

        # --- TEST 1: TRANSLATION ---
        try:
            input("\n[TEST 1] Place robot in clear area. Press ENTER to fire 3-second forward burst...")
        except EOFError:
            self.get_logger().error("CRITICAL ERROR: Cannot read keyboard input!")
            self.get_logger().error("You must run this node using 'ros2 run' in a separate terminal, NOT in the launch file.")
            self.is_calibrating = False
            return

        self.fire_burst(TEST_SPEED, TEST_SPEED, TEST_DURATION)

        dist_str = input("\nMeasure distance traveled. Enter distance in METERS: ")
        try:
            D_actual = float(dist_str)
        except ValueError:
            print("Invalid input. Aborting calibration.")
            self.is_calibrating = False
            return

        factor = D_actual / (TEST_SPEED * TEST_DURATION)
        print(f"✅ SPEED_TO_MPS_FACTOR = {factor:.6f}")

        # --- TEST 2: ROTATION ---
        input("\n[TEST 2] Mark orientation. Press ENTER to fire 3-second clockwise spin...")
        self.fire_burst(TEST_SPEED, -TEST_SPEED, TEST_DURATION)

        ang_str = input("\nMeasure angle rotated. Enter angle in DEGREES (e.g., 90, 180): ")
        try:
            theta_rad = np.deg2rad(float(ang_str))
        except ValueError:
            print("Invalid input. Aborting calibration.")
            self.is_calibrating = False
            return

        L_calc = (TEST_SPEED * factor * TEST_DURATION) / theta_rad
        print(f"✅ Calculated 'L' (Half-Wheelbase) = {L_calc:.6f} meters")

        # --- SAVE & BROADCAST ---
        data = {"SPEED_TO_MPS_FACTOR": factor, "L": L_calc}
        os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
        
        with open(self.calib_file, 'w') as f:
            json.dump(data, f)

        print("\nCalibration saved! Notifying other nodes to reload math...")
        self.update_pub.publish(Empty())
        print("✅ Calibration Complete. You can now use the GUI.\n")
        self.is_calibrating = False

    def send_override(self, left, right):
        msg = Vector3()
        msg.x = float(left)
        msg.y = float(right)
        msg.z = 0.0
        self.override_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()