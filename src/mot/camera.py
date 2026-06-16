import numpy as np
from .models import ObjectState
from .camera_detection import CameraDetection


class CameraSensor:
    """
    Lightweight camera simulator.

    The camera provides:
    - object bearing / azimuth
    - class label
    - classification confidence
    """

    def __init__(
        self,
        azimuth_std_rad: float = 0.015,
        detection_probability: float = 0.9,
        fov_rad: float = np.deg2rad(90.0),
        seed: int = 11,
    ) -> None:
        self.azimuth_std_rad = azimuth_std_rad
        self.detection_probability = detection_probability
        self.fov_rad = fov_rad
        self.rng = np.random.default_rng(seed)

    def measure(self, objects: list[ObjectState]) -> list[CameraDetection]:
        detections = []

        for obj in objects:
            az = np.arctan2(obj.y, obj.x)

            if abs(az) > self.fov_rad / 2:
                continue

            if self.rng.random() > self.detection_probability:
                continue

            noisy_az = az + self.rng.normal(0.0, self.azimuth_std_rad)
            confidence = float(self.rng.uniform(0.75, 0.98))

            detections.append(
                CameraDetection(
                    azimuth=noisy_az,
                    label=obj.label,
                    confidence=confidence,
                    true_id=obj.object_id,
                )
            )

        return detections