from dataclasses import dataclass

@dataclass(frozen=True)
class Environment:
    """Represents the physical parameters of a celestial body/environment."""
    name: str
    gravity: float        # m/s^2
    air_density: float    # kg/m^3

# Pre-configured environments
EARTH = Environment(name="Earth", gravity=9.81, air_density=1.225)
MARS = Environment(name="Mars", gravity=3.72, air_density=0.020)
VENUS = Environment(name="Venus", gravity=8.87, air_density=65.0)
SPACE = Environment(name="Deep Space", gravity=0.0, air_density=0.0)