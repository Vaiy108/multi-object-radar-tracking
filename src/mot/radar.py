import numpy as np
from .models import ObjectState, RadarDetection

class RadarSensor:
    """
    Simple automotive radar detection simulator.

    Measurements:
    - range
    - azimuth
    - radial velocity / range rate

    Includes:
    - Gaussian measurement noise
    - missed detections
    - random clutter / false alarms
    """
    def __init__(
        self,
        range_std_m: float = 0.5,
        azimuth_std_rad: float = 0.01,
        range_rate_std_mps: float = 0.2,
        detection_probability: float = 0.9,
        clutter_rate: int = 3,
        max_range_m: float = 120.0,
        fov_rad: float = np.deg2rad(120.0),
        seed: int = 7,
    ) -> None:
        self.range_std_m = range_std_m
        self.azimuth_std_rad = azimuth_std_rad
        self.range_rate_std_mps = range_rate_std_mps
        self.detection_probability = detection_probability
        self.clutter_rate = clutter_rate
        self.max_range_m = max_range_m
        self.fov_rad = fov_rad
        self.rng = np.random.default_rng(seed)

    def measure(self, objects: list[ObjectState]) -> list[RadarDetection]:
        detections: list[RadarDetection] = []

        for obj in objects:
            px, py, vx, vy = obj.as_vector()
            rng = np.hypot(px, py)
            az = np.arctan2(py, px)

            if rng > self.max_range_m or abs(az) > self.fov_rad / 2:
                continue

            if self.rng.random() > self.detection_probability:
                continue

            radial_velocity = (px * vx + py * vy) / max(rng, 1e-6)
            #Gaussian noise added to range, azimuth, range rate
            detections.append(RadarDetection(
                rng=rng + self.rng.normal(0.0, self.range_std_m), 
                azimuth=az + self.rng.normal(0.0, self.azimuth_std_rad),
                range_rate=radial_velocity + self.rng.normal(0.0, self.range_rate_std_mps),
                true_id=obj.object_id,
            ))

        # Poisson-distributed clutter count around clutter_rate - to create fake detections
        num_clutter = self.rng.poisson(self.clutter_rate)
        for _ in range(num_clutter):
            clutter_rng = self.rng.uniform(5.0, self.max_range_m)
            clutter_az = self.rng.uniform(-self.fov_rad / 2, self.fov_rad / 2)
            clutter_rr = self.rng.uniform(-15.0, 15.0)
            detections.append(RadarDetection(
                rng=clutter_rng,
                azimuth=clutter_az,
                range_rate=clutter_rr,
                true_id=None,
            ))

        return detections
