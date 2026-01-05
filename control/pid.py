"""
PID Controller

Standard PID implementation for velocity control.

Usage:
    from planning import PID

    pid = PID(kp=1.0, ki=0.1, kd=0.01, output_min=-1.0, output_max=1.0)

    # In control loop:
    throttle = pid.compute(desired_velocity - actual_velocity, dt)
"""


class PID:
    """
    PID Controller with anti-windup.

    Attributes:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_min: Minimum output value
        output_max: Maximum output value
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output.

        Args:
            error: Current error (setpoint - measured)
            dt: Time step in seconds

        Returns:
            Control output (clamped to output_min/max)
        """
        if dt <= 0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Sum and clamp output
        output = p + i + d
        output = max(self.output_min, min(self.output_max, output))

        # Anti-windup: clamp integral if output saturated
        if output == self.output_max or output == self.output_min:
            self._integral -= error * dt

        return output

    def reset(self) -> None:
        """Reset integral and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0
