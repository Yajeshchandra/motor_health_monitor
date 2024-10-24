from filterpy.kalman import KalmanFilter
import numpy as np

def setup_kalman_filter():
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.array([[0.], [0.]])  # Initial state (position and velocity)
    kf.F = np.array([[1., 1.],
                     [0., 1.]])    # State transition matrix
    kf.H = np.array([[1., 0.]])    # Measurement matrix
    kf.P *= 1000.0                 # Covariance matrix
    kf.R = 5                       # Measurement noise
    kf.Q = np.eye(2)               # Process noise
    return kf

def apply_kalman_filter(kf, data):
    filtered_data = []
    for z in data:
        kf.predict()
        kf.update(z)
        filtered_data.append(kf.x[0])
    return np.array(filtered_data)

def complementary_filter(a, b, alpha=0.98):
    return alpha * a + (1 - alpha) * b
