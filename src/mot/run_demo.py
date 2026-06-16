from pathlib import Path
from .scenario import create_intersection_scenario, step_objects
from .radar import RadarSensor
from .tracker import MultiObjectTracker, TrackState
from .visualization import plot_scene
from .metrics import compute_rmse_time_aligned

def main():
    dt = 0.1
    num_steps = 250

    objects = create_intersection_scenario()

    radar = RadarSensor(
        range_std_m=0.5,
        azimuth_std_rad=0.01,
        range_rate_std_mps=0.2,
        detection_probability=0.95,
        clutter_rate=1,
        seed=7,
    )

    tracker = MultiObjectTracker(
        dt=dt,
        process_noise_std=0.8,
        #gate_threshold=5.99,
        gate_threshold=30.0,
        confirm_hits=5,
        delete_misses=3,
    )

    truth_history = {obj.object_id: [] for obj in objects}
    track_history = {}

    for step in range(num_steps):
        for obj in objects:
            truth_history[obj.object_id].append([obj.x, obj.y])

        detections = radar.measure(objects)
        tracks = tracker.step(detections, step)
        for track_idx, weights in getattr(tracker, "jpda_weights", {}).items():
            if len(weights) > 1:
                print(f"\nAmbiguous JPDA gate at step {step}, track index {track_idx}")

                for w in weights:
                    print(
                        f"  Detection {w['det_idx']} "
                        f"Probability={w['probability']:.3f}"
                    )

        for trk in tracks:
            if trk.state == TrackState.CONFIRMED:
                track_history.setdefault(trk.track_id, []).append((step, trk.position()))

        step_objects(objects, dt)

    out = Path("outputs/tracking_demo.png")
    plot_scene(truth_history, track_history, out)

    confirmed = [t for t in tracker.tracks if t.state == TrackState.CONFIRMED]
    print(f"Confirmed tracks at end: {len(confirmed)}")
    print(f"Saved plot: {out}")

    # print("\nTracking RMSE summary:")

    # for truth_id, truth_xy in truth_history.items():
    #     best_track_id = None
    #     best_rmse = None

    #     for track_id, track_xy in track_history.items():
    #         rmse = compute_rmse(truth_xy, track_xy)

    #         if rmse is not None and (best_rmse is None or rmse < best_rmse):
    #             best_rmse = rmse
    #             best_track_id = track_id

    #     if best_track_id is not None:
    #         print(f"Truth {truth_id} matched to Track {best_track_id}: RMSE = {best_rmse:.2f} m")
    #     else:
    #         print(f"Truth {truth_id}: no matched track")

    # manual_matches = {
    #     1: 91,
    #     2: 1,
    #     3: 158,
    #     4: 79,
    # }

    # print("\nTime-aligned manual RMSE summary:")

    # for truth_id, track_id in manual_matches.items():
    #     rmse = compute_rmse_time_aligned(
    #         truth_history[truth_id],
    #         track_history[track_id]
    #     )
    #     print(f"Truth {truth_id} matched to Track {track_id}: RMSE = {rmse:.2f} m")


    print("\nConfirmed track summary:")

    for track_id, info in tracker.confirmed_track_log.items():
        print(
            f"Track {track_id}: "
            f"confirmed at step {info['confirmed_at_step']}, "
            f"age at confirmation={info['first_seen_age']}"
        )

    expected_true_tracks = len(truth_history)
    num_confirmed_tracks = len(tracker.confirmed_track_log)
    false_confirmed_tracks = max(0, num_confirmed_tracks - expected_true_tracks)

    print("\nTrack quality summary:")
    print(f"Expected true objects: {expected_true_tracks}")
    print(f"Confirmed tracks: {num_confirmed_tracks}")
    print(f"False confirmed tracks: {false_confirmed_tracks}")

    print("\nFinal Track Existence Probabilities:")

    for trk in tracker.tracks:
        print(
            f"Track {trk.track_id}: "
            f"existence={trk.existence_probability:.2f}, "
            f"hits={trk.hits}, "
            f"misses={trk.misses}"
        )

    # print("\nJPDA Validation Sets:")

    # for track_idx, dets in tracker.jpda_weights.items():

    #     print(f"\nTrack {track_idx}")

    #     for det in dets:

    #         print(
    #             f"Detection {det['det_idx']} "
    #             f"Probability={det['probability']:.3f}"
    #         )

if __name__ == "__main__":
    main()
