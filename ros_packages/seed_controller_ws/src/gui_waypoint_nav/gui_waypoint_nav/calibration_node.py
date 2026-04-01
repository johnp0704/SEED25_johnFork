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
        self.get_logger().info("Kinematic Calibrator Ready.")

        # Subscribes to the GUI button
        self.create_subscription(Empty, '/start_calibration', self.trigger_calib, 10)
        
        # Publishes overrides to the Path Follower, and updates to the network
        self.override_pub = self.create_publisher(Vector3, '/override_motor_cmds', 10)
        self.update_pub = self.create_publisher(Empty, '/calibration_updated', 10)
        
        self.calib_file = os.path.expanduser('~/.ros/seed_calibration.json')
        self.is_calibrating = False

    def trigger_calib(self, msg):
        if self.is_calibrating:
            self.get_logger().warn("Calibration already in progress!")
            return
            
        self.is_calibrating = True
        self.get_logger().warn("\n\n>>> CALIBRATION TRIGGERED. LOOK AT THIS TERMINAL TO INTERACT! <<<\n")
        
        # Run in a separate thread so it doesn't block the ROS spinner while waiting for input()
        threading.Thread(target=self.run_calibration, daemon=True).start()

    def run_calibration(self):
        TEST_SPEED = 20.0
        TEST_DURATION = 3.0

        print("\n" + "="*50)
        print(" DIFFERENTIAL DRIVE KINEMATIC CALIBRATOR")
        print("="*50)

        # --- TEST 1: TRANSLATION ---
        input("\n[TEST 1] Place robot in clear area. Press ENTER to fire 3-second forward burst...")
        self.send_override(TEST_SPEED, TEST_SPEED)
        time.sleep(TEST_DURATION)
        self.send_override(0.0, 0.0)

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
        self.send_override(TEST_SPEED, -TEST_SPEED) # Left fwd, Right rev
        time.sleep(TEST_DURATION)
        self.send_override(0.0, 0.0)

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
        
        # Ensure the .ros directory exists
        os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
        
        with open(self.calib_file, 'w') as f:
            json.dump(data, f)

        print("\nCalibration saved! Notifying other nodes to reload math...")
        self.update_pub.publish(Empty())
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