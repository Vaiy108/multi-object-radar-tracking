from dataclasses import dataclass
import numpy as np

@dataclass
class ObjectState:
    """Ground-truth object state in Cartesian coordinates."""
    x: float
    y: float
    vx: float
    vy: float
    object_id: int
    label: str = "vehicle"

    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.vx, self.vy], dtype=float)

    def step(self, dt: float) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt


@dataclass
class RadarDetection:
    """Radar detection in polar measurement space."""
    rng: float
    azimuth: float
    range_rate: float
    true_id: int | None = None

    def to_cartesian_position(self) -> np.ndarray:
        return np.array([
            self.rng * np.cos(self.azimuth),
            self.rng * np.sin(self.azimuth)
        ], dtype=float)
