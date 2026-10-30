import matplotlib.pyplot as plt
from src.environment import EARTH, MARS
from src.system import Rocket
from src.pid import DiscretePIDController
from src.simulation import SimulationRunner

def plot_telemetry(telemetry, title: str):
    """Utility function to visualize the controller's performance."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot Altitude
    ax1.plot(telemetry.times, telemetry.process_variables, label="Actual Altitude (m)", color="blue")
    ax1.plot(telemetry.times, telemetry.setpoints, label="Target Altitude (m)", color="red", linestyle="--")
    ax1.set_ylabel("Altitude (m)")
    ax1.set_title(f"{title} - System Response")
    ax1.legend()
    ax1.grid(True)

    # Plot Control Signal (Thrust)
    ax2.plot(telemetry.times, telemetry.control_signals, label="Engine Thrust (N)", color="orange")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Thrust (N)")
    ax2.set_title("Control Effort")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    # 1. Define the Environment
    env = EARTH  # Swap to MARS to see how the PID reacts to lower gravity!

    # 2. Define the System (100kg rocket, 0.5 Cd, 1.0m^2 frontal area)
    rocket = Rocket(mass=100.0, drag_coefficient=0.5, cross_section_area=1.0)

    # 3. Define the Controller
    pid = DiscretePIDController(
        kp=150.0,
        ki=50.0,
        kd=200.0,
        output_limits=(rocket.min_thrust, rocket.max_thrust)
    )

    # 4. Initialize and run simulation
    print(f"Starting simulation on {env.name}...")
    sim = SimulationRunner(system=rocket, controller=pid, env=env, dt=0.05)

    # Try to reach and hover at 100 meters for 30 seconds
    telemetry = sim.run(setpoint=100.0, duration=30.0)

    print("Simulation complete. Generating plots...")

    # 5. Analyze results
    plot_telemetry(telemetry, f"Rocket Hover PID on {env.name}")

if __name__ == "__main__":
    main()