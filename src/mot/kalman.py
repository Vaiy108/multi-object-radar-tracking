import numpy as np

class ConstantVelocityKalmanFilter:
    """
    2D constant-velocity Kalman filter.

    State:
        x = [px, py, vx, vy]^T

    Measurement:
        z = [px, py]^T 

    The radar measurement is converted from polar to Cartesian position.
    This keeps Phase 1 simple and C-portable.
    """
    def __init__(self, dt: float, process_noise_std: float, measurement_noise_std: float = 1.0):
        self.dt = dt
        self.q = process_noise_std ** 2
        self.r = measurement_noise_std ** 2

    def F(self) -> np.ndarray:
        dt = self.dt
        return np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)

    def Q(self) -> np.ndarray:
        dt = self.dt
        q = self.q
        return q * np.array([
            [dt**4 / 4, 0, dt**3 / 2, 0],
            [0, dt**4 / 4, 0, dt**3 / 2],
            [dt**3 / 2, 0, dt**2, 0],
            [0, dt**3 / 2, 0, dt**2],
        ], dtype=float)

    def H(self) -> np.ndarray:
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

    def R(self) -> np.ndarray:
        return self.r * np.eye(2)

    def predict(self, x: np.ndarray, P: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        F = self.F()
        x_pred = F @ x
        P_pred = F @ P @ F.T + self.Q()
        return x_pred, P_pred

    def update(self, x: np.ndarray, P: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        H = self.H()
        R = self.R()
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x_upd = x + K @ y
        I = np.eye(len(x))
        P_upd = (I - K @ H) @ P
        return x_upd, P_upd

    def innovation_distance(self, x: np.ndarray, P: np.ndarray, z: np.ndarray) -> float:
        H = self.H()
        R = self.R()
        y = z - H @ x
        S = H @ P @ H.T + R
        return float(y.T @ np.linalg.inv(S) @ y)
