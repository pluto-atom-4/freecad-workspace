#!/usr/bin/env python3
"""
Discrete PID controller for the balance plant.

Pure Python, no Webots dependencies.
Caller supplies measured dt per call (real control-loop fire spacing jitters).
"""


class PlantPID:
    """Discrete PID controller for the balance plant.

    Caller supplies measured dt per call (real control-loop fire spacing jitters
    between 16ms/32ms — never assume a fixed dt).
    """

    def __init__(self, kp, ki, kd, output_min, output_max,
                 integral_min=None, integral_max=None):
        """Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output (lower saturation bound)
            output_max: Maximum output (upper saturation bound)
            integral_min: Minimum integral term (defaults to output_min if not given)
            integral_max: Maximum integral term (defaults to output_max if not given)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        # Default integral bounds to output bounds if not explicitly given
        self.integral_min = integral_min if integral_min is not None else output_min
        self.integral_max = integral_max if integral_max is not None else output_max

        # State: integral accumulator and previous error
        self._integral = 0.0
        self._prev_error = None

        # Component tracking for telemetry logging
        self._last_p_term = 0.0
        self._last_i_term = 0.0
        self._last_d_term = 0.0

    def step(self, error, dt):
        """Perform one PID step.

        Args:
            error: Current error (setpoint - measured value or similar)
            dt: Time delta since last call, in seconds

        Returns:
            Saturated output value

        Raises:
            ValueError: if dt < 0 (negative time delta)

        Note:
            If dt == 0 exactly, returns P-only output (kp * error) and does NOT
            update integral or derivative state.
            If dt < 0, raises ValueError.
        """
        # Validate dt: negative is an error, zero is allowed (special case)
        if dt < 0:
            raise ValueError("dt must be non-negative")

        # Special case: dt == 0 returns P-only output, no state update
        if dt == 0:
            self._last_p_term = self.kp * error
            return self._last_p_term

        # P term
        p_term = self.kp * error

        # I term: accumulate error * dt, then clamp integral state
        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))
        i_term = self.ki * self._integral

        # D term: error-based derivative (not measurement-based)
        # On first call (_prev_error is None), skip derivative term
        if self._prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self._prev_error) / dt

        # Store components for telemetry logging
        self._last_p_term = p_term
        self._last_i_term = i_term
        self._last_d_term = d_term

        # Sum and saturate to output bounds
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        # Update state for next call
        self._prev_error = error

        return output

    def reset(self):
        """Reset controller state (integral and previous error)."""
        self._integral = 0.0
        self._prev_error = None

    def last_components(self):
        """Return (p_term, i_term, d_term) from the most recent step() call.
        Before any step() call, returns (0.0, 0.0, 0.0)."""
        return (self._last_p_term, self._last_i_term, self._last_d_term)
