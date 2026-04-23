"""
dead_reckoning_calibration.py

Runs the robot at a fixed speed, waits for it to fully ramp up via the
Sabertooth trapezoidal profile, then times a fixed-duration straight run.
The user measures the distance traveled and the script saves a calibration
file (dead_reckoning_cal.npz) that the virtual twin GUI reads on startup.

Usage:
    python3 dead_reckoning_calibration.py
"""

import time
import numpy as np
import os

# ---------------------------------------------------------------------------
# Import the motor driver from wherever it lives in your package.
# Adjust the path / package name as needed.
# ---------------------------------------------------------------------------
from sabertooth import SaberToothMotorDriver


# ===========================================================================
# Calibration parameters — edit these if needed
# ===========================================================================
TEST_SPEED      = 40.0   # Sabertooth command unit (0–100)
RAMP_DURATION   = 2.0    # Seconds to let the trapezoidal ramp reach full speed
                         # At accel_rate=80 and call_rate=20 Hz:
                         #   accel_step = 80/20 = 4 units/call
                         #   steps needed = 40/4 = 10 calls = 0.5 s
                         # 2.0 s gives generous margin for any mechanical lag.
RUN_DURATION    = 4.0    # Seconds of constant-speed travel that YOU measure
CALL_RATE_HZ    = 20.0   # Must match SaberToothMotorDriver call_rate_hz
CALL_SLEEP      = 1.0 / CALL_RATE_HZ

OUTPUT_FILE     = "dead_reckoning_cal.npz"
# ===========================================================================


def _spin_motors(motor: SaberToothMotorDriver, duration_sec: float,
                 left: float, right: float) -> None:
    """Call updateMotorSpeed at CALL_RATE_HZ for exactly duration_sec seconds."""
    end = time.monotonic() + duration_sec
    while time.monotonic() < end:
        motor.updateMotorSpeed(left, right)
        time.sleep(CALL_SLEEP)


def _safe_stop(motor: SaberToothMotorDriver) -> None:
    """
    Correct all_motors_off: send the explicit stop byte for EACH motor channel
    rather than a single 0x00 (which only addresses Motor-1 reverse on the
    Sabertooth simplified-serial protocol).

    Motor-1 stop = 64  (mid-point of the 1-127 range)
    Motor-2 stop = 192 (mid-point of the 128-255 range)
    """
    motor._current_left  = 0.0
    motor._current_right = 0.0
    motor._write_motor1(0.0)   # sends byte 64  → Motor-1 stop
    motor._write_motor2(0.0)   # sends byte 192 → Motor-2 stop
    print("ALL MOTORS OFF SENT (both channels)")


def run_calibration() -> None:
    print("=" * 60)
    print("  Dead-Reckoning Calibration Script")
    print("=" * 60)

    # ------------------------------------------------------------------
    # 1. Initialise motors
    # ------------------------------------------------------------------
    try:
        motor = SaberToothMotorDriver(
            motor1_reversed=True,
            motor2_reversed=True,
            accel_rate=80.0,
            decel_rate=120.0,
            call_rate_hz=CALL_RATE_HZ,
        )
        print("Motors initialised successfully.\n")
    except Exception as exc:
        print(f"ERROR — Failed to initialise motors: {exc}")
        return

    # ------------------------------------------------------------------
    # 2. Ramp-up phase (not measured)
    # ------------------------------------------------------------------
    print(f"Step 1 of 3 — Ramp-up phase ({RAMP_DURATION:.1f} s)")
    print("  Place the robot on an open floor with at least 3 m of clear space.")
    print("  The robot will accelerate to cruise speed. Do NOT measure yet.\n")
    input("  Press ENTER when ready to begin ramp-up...")

    print(f"  Ramping up to speed {TEST_SPEED} ...")
    _spin_motors(motor, RAMP_DURATION, TEST_SPEED, TEST_SPEED)
    print("  Ramp-up complete. Robot should now be at cruise speed.")

    # ------------------------------------------------------------------
    # 3. Measured run phase
    # ------------------------------------------------------------------
    print(f"\nStep 2 of 3 — Measured run ({RUN_DURATION:.1f} s)")
    print("  Mark the robot's CURRENT position (front axle or a reference point).")
    input("  Press ENTER to start the timed measurement run...")

    print(f"  Running for {RUN_DURATION:.1f} s — WATCH the robot!")
    _spin_motors(motor, RUN_DURATION, TEST_SPEED, TEST_SPEED)

    _safe_stop(motor)
    print("  Motors stopped. Mark the robot's END position.")

    # ------------------------------------------------------------------
    # 4. User measurement input
    # ------------------------------------------------------------------
    print(f"\nStep 3 of 3 — Enter measurement")
    while True:
        try:
            measured_distance = float(
                input("  Enter the measured distance between the two marks (metres): ")
            )
            if measured_distance <= 0:
                print("  Distance must be positive. Try again.")
                continue
            break
        except ValueError:
            print("  Invalid input — please enter a number.")

    # ------------------------------------------------------------------
    # 5. Compute and save calibration constants
    # ------------------------------------------------------------------
    velocity_mps      = measured_distance / RUN_DURATION
    cmd_to_mps_ratio  = velocity_mps / TEST_SPEED

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
        ramp_duration  = RAMP_DURATION,
        run_duration   = RUN_DURATION,
    )
    print(f"\nCalibration saved → {os.path.abspath(OUTPUT_FILE)}")
    print("The GUI virtual twin will load this file automatically on next launch.")


if __name__ == "__main__":
    run_calibration()