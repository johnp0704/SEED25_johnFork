"""
dead_reckoning_calibration.py

Runs the robot at a fixed speed using a strict square-wave profile 
(instant start, instant stop). The robot remains entirely at rest 
until the user presses ENTER, runs for the precise duration, and halts.
The user measures the distance traveled and the script saves a calibration
file (dead_reckoning_cal.npz) that the virtual twin GUI reads on startup.

Usage:
    python3 dead_reckoning_calibration.py
"""

import sys
import time
import numpy as np
import os

# ---------------------------------------------------------------------------
# Import the custom calibration motor driver (no trapezoidal ramping)
# ---------------------------------------------------------------------------
from sabertooth import SaberToothCalDriver as st

# ===========================================================================
# Calibration parameters — edit these if needed
# ===========================================================================
TEST_SPEED   = 40.0  # Sabertooth command unit (0–100)
RUN_DURATION = 4.0   # Seconds of constant-speed travel that YOU measure
OUTPUT_FILE  = "dead_reckoning_cal.npz"
# ===========================================================================

def run_calibration() -> None:
    print("=" * 60)
    print("  Square-Profile Dead-Reckoning Calibration")
    print("=" * 60)

    # ------------------------------------------------------------------
    # 1. Initialize motors (Ensures zero state immediately)
    # ------------------------------------------------------------------
    try:
        motor = st(motor1_reversed=True, motor2_reversed=True)
        print("Motors initialized successfully. Robot is AT REST.\n")
    except Exception as exc:
        print(f"ERROR — Failed to initialize motors: {exc}")
        return

    # Wrap the entire operation in a try/except to catch Ctrl+C instantly
    try:
        # ------------------------------------------------------------------
        # 2. Measured run phase
        # ------------------------------------------------------------------
        print(f"Step 1 of 2 — Measured run ({RUN_DURATION:.1f} s)")
        print("  Mark the robot's CURRENT position (front axle or a reference point).")
        print("  WARNING: The robot will lurch to speed instantly (no ramping).")
        input("  Press ENTER to start the timed measurement run...")

        print(f"  Running for {RUN_DURATION:.1f} s — WATCH the robot!")
        
        # Instant start (Square wave high)
        motor.set_speed(TEST_SPEED, TEST_SPEED)

        # Precise timing loop
        end_time = time.monotonic() + RUN_DURATION
        while time.monotonic() < end_time:
            time.sleep(0.01) # Tight loop keeps CPU usage low but reacts quickly

        # Instant stop (Square wave low)
        motor.stop()
        print("  Motors stopped. Mark the robot's END position.")

        # ------------------------------------------------------------------
        # 3. User measurement input
        # ------------------------------------------------------------------
        print(f"\nStep 2 of 2 — Enter measurement")
        while True:
            try:
                measured_distance = float(
                    input("  Enter the measured distance between the two marks (meters): ")
                )
                if measured_distance <= 0:
                    print("  Distance must be positive. Try again.")
                    continue
                break
            except ValueError:
                print("  Invalid input — please enter a number.")

        # ------------------------------------------------------------------
        # 4. Compute and save calibration constants
        # ------------------------------------------------------------------
        velocity_mps     = measured_distance / RUN_DURATION
        cmd_to_mps_ratio = velocity_mps / TEST_SPEED

        print("\n--- Results ---")
        print(f"  Measured distance : {measured_distance:.3f} m")
        print(f"  Run duration      : {RUN_DURATION:.1f} s")
        print(f"  Velocity          : {velocity_mps:.4f} m/s  at command {TEST_SPEED}")
        print(f"  cmd_to_mps ratio  : {cmd_to_mps_ratio:.6f}  (m/s per command unit)")

        np.savez(
            OUTPUT_FILE,
            test_command   = TEST_SPEED,
            velocity_mps   = velocity_mps,
            ratio          = cmd_to_mps_ratio,
            run_duration   = RUN_DURATION,
        )
        print(f"\nCalibration saved → {os.path.abspath(OUTPUT_FILE)}")
        print("The GUI virtual twin will load this file automatically on next launch.")

    except KeyboardInterrupt:
        # ------------------------------------------------------------------
        # SAFETY CATCH: Triggers immediately on Ctrl+C
        # ------------------------------------------------------------------
        motor.stop()
        print("\n\n[!] EMERGENCY STOP: Calibration aborted by user (Ctrl+C).")
        print("Motors halted safely.")
        sys.exit(1)
    finally:
        # ------------------------------------------------------------------
        # REDUNDANT SAFETY CATCH: Triggers if the script crashes unexpectedly
        # ------------------------------------------------------------------
        motor.stop()

if __name__ == "__main__":
    run_calibration()