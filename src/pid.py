class DiscretePIDController:
    def __init__(self, kp: float, ki: float, kd: float, output_limits: tuple[float, float] = (None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits
        # Internal state
        self._prev_error: float = 0.0
        self._integral: float = 0.0

    def compute(self, setpoint: float, process_variable: float, dt: float) -> float:
        """Computes the control signal.
        :param setpoint: Desired target (e.g., target altitude).
        :param process_variable: Current state (e.g., current altitude).
        :param dt: Time delta since last update."""
        if dt <= 0:
            raise ValueError("Time step (dt) must be strictly positive.")

        error = setpoint - process_variable

        # Proportional term
        p_out = self.kp * error

        # Integral term (with basic anti-windup constraint via output limits if provided)
        self._integral += error * dt
        i_out = self.ki * self._integral

        # Derivative term
        derivative = (error - self._prev_error) / dt
        d_out = self.kd * derivative

        output = p_out + i_out + d_out

        # Clamp output to physical limits (e.g., max engine thrust)
        if self.min_out is not None:
            output = max(self.min_out, output)
        if self.max_out is not None:
            output = min(self.max_out, output)

        # Update state
        self._prev_error = error
        return output

    def reset(self):
        """Clears the internal state, useful between simulation runs."""
        self._prev_error = 0.0
        self._integral = 0.0

    def tune(self, method: str = "ziegler-nichols"):
        """Placeholder for an auto-tuning algorithm."""
        if method == "ziegler-nichols":
            # TODO: Implement closed-loop oscillation logic
            pass
        else:
            raise NotImplementedError(f"Tuning method '{method}' not implemented.")