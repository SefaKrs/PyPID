from dataclasses import dataclass, field
from src.environment import Environment
from src.system import SimulatedSystem
from src.pid import DiscretePIDController

@dataclass
class Telemetry:
    """Stores the timeseries data of a simulation run."""
    times: list[float] = field(default_factory=list)
    process_variables: list[float] = field(default_factory=list)
    setpoints: list[float] = field(default_factory=list)
    control_signals: list[float] = field(default_factory=list)

class SimulationRunner:
    def __init__(self,
                 system: SimulatedSystem,
                 controller: DiscretePIDController,
                 env: Environment,
                 dt: float = 0.05):
        """Orchestrates the control loop."""
        self.system = system
        self.controller = controller
        self.env = env
        self.dt = dt

    def run(self, setpoint: float, duration: float) -> Telemetry:
        """Executes the simulation loop for a given duration."""
        telemetry = Telemetry()
        current_time = 0.0
        # Ensure PID is clean before a run
        self.controller.reset()

        while current_time <= duration:
            # 1. Measure the current state of the system
            current_pv = getattr(self.system, 'altitude', 0.0)

            # 2. Compute the control action based on the error
            control_signal = self.controller.compute(setpoint, current_pv, self.dt)

            # 3. Apply the control action to the physical system
            self.system.update(control_signal, self.dt, self.env)

            # 4. Record telemetry
            telemetry.times.append(current_time)
            telemetry.process_variables.append(current_pv)
            telemetry.setpoints.append(setpoint)
            telemetry.control_signals.append(control_signal)

            # 5. Advance time
            current_time += self.dt

        return telemetry