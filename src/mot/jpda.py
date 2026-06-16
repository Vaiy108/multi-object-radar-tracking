import numpy as np


def association_probabilities(distances):
    """
    Convert Mahalanobis distances into normalized association probabilities.

    Lower distance = higher probability.
    """
    if len(distances) == 0:
        return []

    likelihoods = np.exp(-0.5 * np.array(distances))
    probabilities = likelihoods / np.sum(likelihoods)

    return probabilities.tolist()


def weighted_measurement(det_positions, weights):
    """
    Compute JPDA-style weighted measurement.

    det_positions:
        list of np.array([x, y])

    weights:
        list of dictionaries:
        {
            "det_idx": int,
            "probability": float
        }
    """
    z_bar = np.zeros(2)

    for w in weights:
        det_idx = w["det_idx"]
        prob = w["probability"]
        z_bar += prob * det_positions[det_idx]

    return z_bar