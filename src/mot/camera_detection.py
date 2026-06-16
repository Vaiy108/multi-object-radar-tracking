from dataclasses import dataclass


@dataclass
class CameraDetection:
    azimuth: float
    label: str
    confidence: float
    true_id: int | None = None