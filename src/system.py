from abc import ABC, abstractmethod
from src.environment import Environment

class SimulatedSystem(ABC):
    @abstractmethod
    def update(self, control_input: float, dt: float, env: Environment) -> float:
        """Updates the system state based on control input and time step.
        Returns the primary variable we are trying to control (e.g., altitude)."""
        pass

class Rocket(SimulatedSystem):
    def __init__(self, mass: float, drag_coefficient: float, cross_section_area: float):
        self.mass = mass                  # kg
        self.cd = drag_coefficient
        self.area = cross_section_area    # m^2
        self.min_thrust = 0.0             # N
        self.max_thrust = 3000.0          # N

        # State variables
        self.altitude: float = 0.0
        self.velocity: float = 0.0

    def update(self, control_input: float, dt: float, env: Environment) -> float:
        """Calculates 1D kinematics (vertical Z axis).
        control_input here is the Thrust (Newtons)."""
        # F_gravity = m * g
        f_gravity = self.mass * env.gravity

        # F_drag = 0.5 * rho * v^2 * Cd * A
        # Drag always opposes the direction of velocity
        direction = 1 if self.velocity > 0 else -1
        f_drag = 0.5 * env.air_density * (self.velocity ** 2) * self.cd * self.area * direction

        # F_net = F_thrust - F_gravity - F_drag
        f_net = control_input - f_gravity - f_drag

        # a = F / m
        acceleration = f_net / self.mass

        # Euler integration
        self.velocity += acceleration * dt
        self.altitude += self.velocity * dt

        # Prevent falling through the ground
        if self.altitude <= 0.0:
            self.altitude = 0.0
            if self.velocity < 0:
                self.velocity = 0.0

        return self.altitude