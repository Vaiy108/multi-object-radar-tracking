import numpy as np
from .tracker import Track, TrackState
from .camera_detection import CameraDetection


def associate_camera_to_radar_tracks(
    tracks: list[Track],
    camera_detections: list[CameraDetection],
    azimuth_gate_rad: float = 0.08,
) -> list[dict]:
    """
    Lightweight radar-camera fusion.

    Radar provides:
    - position
    - velocity
    - track ID
    - existence probability

    Camera provides:
    - class label
    - classification confidence
    - bearing / azimuth

    Fusion:
    - compare radar track azimuth with camera detection azimuth
    - attach camera class to the nearest gated radar track
    """

    fused_objects = []

    for trk in tracks:
        if trk.state != TrackState.CONFIRMED:
            continue

        px, py = trk.position()
        track_azimuth = np.arctan2(py, px)

        best_det = None
        best_error = None

        for det in camera_detections:
            error = abs(_wrap_angle(track_azimuth - det.azimuth))

            if error <= azimuth_gate_rad:
                if best_error is None or error < best_error:
                    best_error = error
                    best_det = det

        if best_det is not None:
            trk.class_label = best_det.label
            trk.class_confidence = best_det.confidence

        fused_objects.append(
            {
                "track_id": trk.track_id,
                "position": trk.position(),
                "velocity": trk.x[2:4].copy(),
                "existence_probability": trk.existence_probability,
                "class_label": trk.class_label,
                "class_confidence": trk.class_confidence,
            }
        )

    return fused_objects


def _wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi