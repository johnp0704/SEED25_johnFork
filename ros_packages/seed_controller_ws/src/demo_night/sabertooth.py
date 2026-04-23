"""
sabertooth_cal.py

A stripped-down Sabertooth driver for use ONLY during dead-reckoning
calibration.  Unlike the main sabertooth.py, this class sends commands
directly to the hardware with NO trapezoidal velocity ramping.  The robot
will start and stop abruptly, which is exactly what calibration needs so
that the timed run precisely reflects a square-wave speed profile.

DO NOT use this class in normal operation — use sabertooth.py instead.

Relevant documentation:
https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf
"""

import serial
import time


def _linear_map_constrain_int(value, from_low, from_high, to_low, to_high):
    factor    = (value - from_low) / (from_high - from_low)
    mapped    = (to_high - to_low) * factor + to_low
    return round(min(to_high, max(to_low, mapped)))


class SaberToothCalDriver:
    """
    Minimal Sabertooth driver with no velocity ramping.

    Sends the raw simplified-serial byte immediately for both channels.
    Motor 1 (right wheel) uses the 1–127 byte range.
    Motor 2 (left  wheel) uses the 128–255 byte range.
    """

    def __init__(self, motor1_reversed: bool, motor2_reversed: bool,
                 port: str = "/dev/ttyTHS0"):
        self.motor1_reversed = motor1_reversed
        self.motor2_reversed = motor2_reversed

        self._ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        # Give the Sabertooth time to come up after serial open
        time.sleep(1)

    # ------------------------------------------------------------------
    # Low-level channel writes
    # ------------------------------------------------------------------

    def _write_motor1(self, speed: float) -> None:
        """Motor 1 (right). speed in [-100, 100]. Byte range 1–127, 64=stop."""
        if self.motor1_reversed:
            speed = -speed
        if speed < 0:
            val = _linear_map_constrain_int(100 + speed, 0, 100, 1, 64)
        elif speed > 0:
            val = _linear_map_constrain_int(speed, 0, 100, 64, 127)
        else:
            val = 64  # stop
        self._ser.write([val])

    def _write_motor2(self, speed: float) -> None:
        """Motor 2 (left). speed in [-100, 100]. Byte range 128–255, 192=stop."""
        if self.motor2_reversed:
            speed = -speed
        if speed < 0:
            val = _linear_map_constrain_int(100 + speed, 0, 100, 128, 192)
        elif speed > 0:
            val = _linear_map_constrain_int(speed, 0, 100, 192, 255)
        else:
            val = 192  # stop
        self._ser.write([val])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_speed(self, left: float, right: float) -> None:
        """
        Immediately command both wheels. No ramping.
        left / right are in Sabertooth command units [-100, 100].
        Positive = forward.
        """
        self._write_motor2(left)
        self._write_motor1(right)

    def stop(self) -> None:
        """
        Immediately halt both motors by sending the explicit stop byte
        for each channel independently.
        Byte 64  = Motor-1 stop (centre of the 1–127 range).
        Byte 192 = Motor-2 stop (centre of the 128–255 range).
        Sending a single 0x00 only addresses Motor-1 reverse — NOT a broadcast stop.
        """
        self._write_motor1(0.0)   # → byte 64
        self._write_motor2(0.0)   # → byte 192

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass