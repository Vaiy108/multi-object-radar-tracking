import numpy as np

# def compute_rmse(truth_xy, track_xy):
#     n = min(len(truth_xy), len(track_xy))
#     if n == 0:
#         return None

#     truth = np.array(truth_xy[-n:])
#     track = np.array(track_xy[-n:])

#     error = truth - track
#     rmse = np.sqrt(np.mean(np.sum(error**2, axis=1)))
#     return float(rmse)
def compute_rmse_time_aligned(truth_history, track_history):
    errors = []

    truth_by_step = {
        step: np.array(pos)
        for step, pos in enumerate(truth_history)
    }

    for step, track_pos in track_history:
        if step in truth_by_step:
            error = truth_by_step[step] - np.array(track_pos)
            errors.append(np.sum(error**2))

    if not errors:
        return None

    return float(np.sqrt(np.mean(errors)))