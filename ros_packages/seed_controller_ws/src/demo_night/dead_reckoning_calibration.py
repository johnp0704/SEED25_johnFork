import time
import numpy as np
import os
from sabertooth import SaberToothMotorDriver

def run_calibration():
    # Initialize Sabertooth (adjust True/False based on your wiring)
    try:
        motor = SaberToothMotorDriver(motor1_reversed=True, motor2_reversed=True, call_rate_hz=20.0)
        print("Motors initialized.")
    except Exception as e:
        print(f"Failed to initialize motors: {e}")
        return

    # Calibration parameters
    test_speed = 40.0      # Sabertooth command unit (0-100)
    duration_sec = 3.0     # Time to drive forward
    call_rate = 0.05       # 20 Hz loop

    print(f"\n--- Linear Odometry Calibration ---")
    print(f"The robot will drive FORWARD at speed {test_speed} for {duration_sec} seconds.")
    input("Clear the path, grab your tape measure, and press ENTER to begin...")

    start_time = time.time()
    while (time.time() - start_time) < duration_sec:
        motor.updateMotorSpeed(test_speed, test_speed)
        time.sleep(call_rate)
    
    motor.all_motors_off()

    measured_distance = float(input("\nEnter the measured distance traveled (in meters): "))
    
    # Calculate empirical velocity
    velocity_mps = measured_distance / duration_sec
    cmd_to_mps_ratio = velocity_mps / test_speed

    print(f"\nResults:")
    print(f"Velocity: {velocity_mps:.3f} m/s at command {test_speed}")
    print(f"Ratio: {cmd_to_mps_ratio:.5f} (m/s) per command unit")

    # Save to file for the virtual twin to use
    save_path = "dead_reckoning_cal.npz"
    np.savez(save_path, 
             test_command=test_speed, 
             velocity_mps=velocity_mps, 
             ratio=cmd_to_mps_ratio)
    
    print(f"Calibration data saved to {os.path.abspath(save_path)}")

if __name__ == "__main__":
    run_calibration()