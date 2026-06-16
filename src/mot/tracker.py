from dataclasses import dataclass, field
from enum import Enum
import numpy as np
from scipy.optimize import linear_sum_assignment

from .models import RadarDetection
from .kalman import ConstantVelocityKalmanFilter

class TrackState(str, Enum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    DELETED = "deleted"

@dataclass
class Track:
    track_id: int
    x: np.ndarray
    P: np.ndarray
    state: TrackState = TrackState.TENTATIVE
    hits: int = 1
    misses: int = 0
    age: int = 1
    history: list[np.ndarray] = field(default_factory=list)
    confirmed_at_step: int | None = None

    def position(self) -> np.ndarray:
        return self.x[:2].copy()

class MultiObjectTracker:
    """
    Multi-object tracker with:
    - Kalman prediction
    - Chi-square gating
    - GNN/Hungarian assignment
    - tentative / confirmed / deleted track lifecycle
    """
    def __init__(
        self,
        dt: float,
        process_noise_std: float = 1.5,
        gate_threshold: float = 9.21,
        confirm_hits: int = 3,
        delete_misses: int = 5,
    ) -> None:
        self.kf = ConstantVelocityKalmanFilter(dt, process_noise_std)
        self.gate_threshold = gate_threshold
        self.confirm_hits = confirm_hits
        self.delete_misses = delete_misses
        self.tracks: list[Track] = []
        self.next_track_id = 1
        self.confirmed_track_log = {}
        self.validation_sets = {}
        self.jpda_weights = {}

    def step(self, detections: list[RadarDetection], current_step: int) -> list[Track]:
        # 1. Predict all tracks
        for trk in self.tracks:
            if trk.state != TrackState.DELETED:
                trk.x, trk.P = self.kf.predict(trk.x, trk.P)
                trk.age += 1

        # 2. Associate detections to tracks
        det_positions = [d.to_cartesian_position() for d in detections]
        matches, unmatched_tracks, unmatched_dets = self._associate(det_positions)

        # 3. Update matched tracks - Kalman update
        for track_idx, det_idx in matches:
            trk = self.tracks[track_idx]
            z = det_positions[det_idx]
            trk.x, trk.P = self.kf.update(trk.x, trk.P, z)
            trk.hits += 1 #Track Management
            trk.misses = 0
            trk.history.append(trk.position())

            if trk.state == TrackState.TENTATIVE and trk.hits >= self.confirm_hits:
                trk.state = TrackState.CONFIRMED
                trk.confirmed_at_step = current_step

                self.confirmed_track_log[trk.track_id] = {
                    "confirmed_at_step": trk.confirmed_at_step,
                    "first_seen_age": trk.age,
                }

        # 4. Age unmatched tracks
        for track_idx in unmatched_tracks:
            trk = self.tracks[track_idx]
            trk.misses += 1  #Track Management
            trk.history.append(trk.position())
            if trk.misses >= self.delete_misses:
                trk.state = TrackState.DELETED

        # 5. Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            z = det_positions[det_idx]
            self._create_track(z)

        # 6. Remove deleted tracks
        self.tracks = [t for t in self.tracks if t.state != TrackState.DELETED]

        return self.tracks

    def _associate(self, det_positions: list[np.ndarray]):

        self.validation_sets = {}
        self.jpda_weights = {}
        active_indices = [i for i, t in enumerate(self.tracks) if t.state != TrackState.DELETED]

        if not active_indices:
            return [], [], list(range(len(det_positions)))

        if not det_positions:
            return [], active_indices, []

        #association cost matrix    

        cost = np.full((len(active_indices), len(det_positions)), fill_value=1e6)

        for row, track_idx in enumerate(active_indices):
            trk = self.tracks[track_idx]
            for col, z in enumerate(det_positions):
                d2 = self.kf.innovation_distance(trk.x, trk.P, z) # Gating
                if not hasattr(self, "validation_sets"):
                    self.validation_sets = {}

                self.validation_sets.setdefault(track_idx, [])

                if d2 <= self.gate_threshold:
                    cost[row, col] = d2
                    self.validation_sets[track_idx].append(
                        {
                            "det_idx": col,
                            "distance": d2,
                        }
                    )

        rows, cols = linear_sum_assignment(cost) #Hungarian algorithm /Assignment

        matches = []
        matched_track_indices = set()
        matched_det_indices = set()

        for r, c in zip(rows, cols):
            if cost[r, c] < 1e6:
                track_idx = active_indices[r]
                matches.append((track_idx, c))
                matched_track_indices.add(track_idx)
                matched_det_indices.add(c)

        unmatched_tracks = [i for i in active_indices if i not in matched_track_indices]
        unmatched_dets = [j for j in range(len(det_positions)) if j not in matched_det_indices]

        
        # JPDA probability calculation
        from .jpda import association_probabilities

        self.jpda_weights = {}

        for track_idx, detections in self.validation_sets.items():

            distances = [
                d["distance"]
                for d in detections
            ]

            probs = association_probabilities(
                distances
            )

            self.jpda_weights[track_idx] = []

            for det, prob in zip(detections, probs):

                self.jpda_weights[track_idx].append(
                    {
                        "det_idx": det["det_idx"],
                        "probability": prob,
                    }
                )

        return matches, unmatched_tracks, unmatched_dets

        


    def _create_track(self, z: np.ndarray) -> None:
        x0 = np.array([z[0], z[1], 0.0, 0.0], dtype=float)
        #P0 = np.diag([4.0, 4.0, 25.0, 25.0])
        P0 = np.diag([1.0, 1.0, 9.0, 9.0])
        trk = Track(track_id=self.next_track_id, x=x0, P=P0)
        trk.history.append(trk.position())
        self.next_track_id += 1
        self.tracks.append(trk)
