
#

# src/ekf_2d.py
import numpy as np

class EKF2D:
    """
    Simple linear Kalman Filter for 2D position and velocity:
    state: [x, y, vx, vy]^T
    control input: acceleration in body/world frame (ax, ay)
    """
    def __init__(self, dt=0.05, process_var=1e-2, meas_var=1.0):
        self.dt = dt
        # State vector
        self.x = np.zeros((4,1), dtype=np.float32)
        # Covariance
        self.P = np.eye(4, dtype=np.float32) * 1.0
        # Transition matrix (constant velocity model)
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1,  0],
                           [0, 0, 0,  1]], dtype=np.float32)
        # Control matrix (acceleration -> affects velocities)
        self.B = np.array([[0.5*dt*dt, 0],
                           [0, 0.5*dt*dt],
                           [dt, 0],
                           [0, dt]], dtype=np.float32)
        # Measurement matrix (we measure x,y)
        self.H = np.array([[1,0,0,0],
                           [0,1,0,0]], dtype=np.float32)
        # Process noise covariance
        q = process_var
        self.Q = np.eye(4, dtype=np.float32) * q
        # Measurement noise covariance
        r = meas_var
        self.R = np.eye(2, dtype=np.float32) * r

    def set_dt(self, dt):
        self.dt = dt
        self.F[0,2] = dt
        self.F[1,3] = dt
        self.B = np.array([[0.5*dt*dt, 0],
                           [0, 0.5*dt*dt],
                           [dt, 0],
                           [0, dt]], dtype=np.float32)

    def predict(self, acc=(0.0, 0.0)):
        u = np.array([[acc[0]], [acc[1]]], dtype=np.float32)
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.copy()

    def update_position(self, meas_x, meas_y):
        z = np.array([[meas_x], [meas_y]], dtype=np.float32)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0], dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P
        return self.x.copy()

    def get_state(self):
        return self.x.flatten().tolist()  # [x,y,vx,vy]
