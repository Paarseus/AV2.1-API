#!/usr/bin/env python3
"""
Vehicle Actuator Interface via Serial

Communicates with Teensy 4.1 CAN Master over USB Serial.

Protocol:
    E 1|0      -> estop on/off
    T 0..1     -> throttle
    M N|D|S|R  -> mode (Neutral/Drive/Sport/Reverse)
    B 0..1     -> brake
    S -1..1    -> steer (left -, right +)
"""

from __future__ import annotations
import logging
import threading
import time
from typing import Optional
import serial

logger = logging.getLogger(__name__)

VALID_MODES = ("N", "D", "S", "R")


class VehicleActuator:
    """Vehicle actuator interface via USB Serial."""

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self.open()

    def open(self):
        """Open serial connection."""
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(1.5)  # Teensy USB CDC setup time
        self._ser.reset_input_buffer()

    def close(self):
        """Close serial connection."""
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def __enter__(self) -> "VehicleActuator":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _send(self, cmd: str):
        """Send command (thread-safe)."""
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        with self._lock:
            self._ser.write((cmd + "\n").encode("ascii"))

    def set_throttle(self, value: float):
        """Set throttle 0..1."""
        self._send(f"T {max(0.0, min(1.0, float(value))):.3f}")

    def set_brake(self, value: float):
        """Set brake 0..1."""
        self._send(f"B {max(0.0, min(1.0, float(value))):.3f}")

    def set_mode(self, mode: str):
        """Set drive mode: N, D, S, or R."""
        m = mode.upper()
        if m not in VALID_MODES:
            raise ValueError(f"Invalid mode '{mode}', use: {VALID_MODES}")
        self._send(f"M {m}")

    def estop(self, on: bool = True):
        """Emergency stop."""
        self._send(f"E {1 if on else 0}")

    def set_steer_norm(self, value: float):
        """Set steering -1..1 (left=-1, right=+1)."""
        self._send(f"S {max(-1.0, min(1.0, float(value))):.3f}")

    def set_steer_deg(self, degrees: float, mech_limit_deg: float = 28.0):
        """Set steering in degrees. Converts to normalized using mechanical limit."""
        d = max(-mech_limit_deg, min(mech_limit_deg, float(degrees)))
        self.set_steer_norm(d / mech_limit_deg)
