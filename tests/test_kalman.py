import numpy as np
from src.mot.kalman import ConstantVelocityKalmanFilter

def test_kalman_prediction_shape():
    kf = ConstantVelocityKalmanFilter(dt=0.1, process_noise_std=1.0)
    x = np.array([0, 0, 1, 1], dtype=float)
    P = np.eye(4)
    xp, Pp = kf.predict(x, P)
    assert xp.shape == (4,)
    assert Pp.shape == (4, 4)
