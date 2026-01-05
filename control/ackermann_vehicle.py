"""
Ackermann Vehicle Controller

Wraps vehicle actuator with:
- Vehicle geometry (wheelbase, steering limits)
- Unit conversions (radians <-> degrees <-> normalized)
- Sign conventions
- Bicycle model kinematics

Usage:
    from control import AckermannVehicle
    from actuators import VehicleActuator

    actuator = VehicleActuator(port="/dev/ttyACM0")
    vehicle = AckermannVehicle(actuator, wheelbase=1.23, max_steering_deg=28.0)

    # Send steering command (radians in, hardware command out)
    vehicle.steer(steering_rad)

    # Predict motion (for trajectory planning)
    new_x, new_y, new_yaw = vehicle.predict(x, y, yaw, v, steering_rad, dt)
"""

import numpy as np
from typing import Tuple, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from actuators import VehicleActuator


class AckermannVehicle:
    """
    Ackermann vehicle controller with bicycle model kinematics.

    Handles all conversions between controller output (radians)
    and actuator input (normalized).
    """

    def __init__(
        self,
        actuator: Optional['VehicleActuator'] = None,
        wheelbase: float = 1.23,
        max_steering_deg: float = 28.0,
        steering_sign: int = -1
    ):
        """
        Args:
            actuator: VehicleActuator instance (can be None for kinematics-only use)
            wheelbase: Distance between front and rear axles (meters)
            max_steering_deg: Maximum steering angle (degrees)
            steering_sign: Sign convention (-1 if hardware is opposite to standard)
        """
        self.actuator = actuator
        self.wheelbase = wheelbase
        self.max_steering_deg = max_steering_deg
        self.max_steering_rad = np.deg2rad(max_steering_deg)
        self.steering_sign = steering_sign

    # ─────────────────────────────────────────────────────────────
    # Commands
    # ─────────────────────────────────────────────────────────────

    def steer(self, steering_rad: float) -> float:
        """
        Send steering command.

        Args:
            steering_rad: Desired steering angle in radians
                          (positive = left in standard convention)

        Returns:
            Actual commanded angle in degrees (after limits and sign)
        """
        # Clip to limits
        steering_rad = np.clip(steering_rad, -self.max_steering_rad, self.max_steering_rad)

        # Convert to degrees
        steering_deg = np.rad2deg(steering_rad)

        # Apply hardware sign convention
        steering_deg *= self.steering_sign

        # Send to actuator
        if self.actuator:
            self.actuator.set_steer_deg(steering_deg)

        return steering_deg

    def throttle(self, value: float):
        """Send throttle command (0.0 to 1.0)"""
        if self.actuator:
            self.actuator.set_throttle(np.clip(value, 0.0, 1.0))

    def brake(self, value: float):
        """Send brake command (0.0 to 1.0)"""
        if self.actuator:
            self.actuator.set_brake(np.clip(value, 0.0, 1.0))

    def stop(self):
        """Emergency stop - zero throttle, full brake"""
        if self.actuator:
            self.actuator.set_throttle(0.0)
            self.actuator.set_brake(1.0)

    def set_mode(self, mode: str):
        """Set drive mode (N, D, S, R)"""
        if self.actuator:
            self.actuator.set_mode(mode)

    def estop(self):
        """Trigger emergency stop"""
        if self.actuator:
            self.actuator.estop()

    # ─────────────────────────────────────────────────────────────
    # Kinematics (Bicycle Model)
    # ─────────────────────────────────────────────────────────────

    def predict(
        self,
        x: float,
        y: float,
        yaw: float,
        velocity: float,
        steering_rad: float,
        dt: float
    ) -> Tuple[float, float, float]:
        """
        Predict next position using bicycle model.

        Args:
            x, y: Current position (meters)
            yaw: Current heading (radians)
            velocity: Current speed (m/s)
            steering_rad: Steering angle (radians)
            dt: Time step (seconds)

        Returns:
            (new_x, new_y, new_yaw)
        """
        if abs(velocity) < 1e-6:
            return x, y, yaw

        # Bicycle model: yaw_rate = v * tan(steering) / wheelbase
        yaw_rate = velocity * np.tan(steering_rad) / self.wheelbase

        # Euler integration
        new_yaw = yaw + yaw_rate * dt
        new_x = x + velocity * np.cos(new_yaw) * dt
        new_y = y + velocity * np.sin(new_yaw) * dt

        return new_x, new_y, new_yaw

    def turning_radius(self, steering_rad: float) -> float:
        """
        Calculate turning radius for given steering angle.

        Returns:
            Turning radius in meters (inf for straight)
        """
        if abs(steering_rad) < 1e-6:
            return float('inf')
        return abs(self.wheelbase / np.tan(steering_rad))

    @property
    def min_turning_radius(self) -> float:
        """Minimum turning radius at max steering angle"""
        return self.wheelbase / np.tan(self.max_steering_rad)
