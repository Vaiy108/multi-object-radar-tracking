import numpy as np


def association_probabilities(distances):
    """
    Convert Mahalanobis distances into
    normalized association probabilities.

    Lower distance = higher probability.
    """

    if len(distances) == 0:
        return []

    likelihoods = np.exp(-0.5 * np.array(distances))

    probabilities = likelihoods / np.sum(likelihoods)

    return probabilities.tolist()