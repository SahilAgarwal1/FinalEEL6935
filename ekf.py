import numpy as np


# test implementations of EKF for slam
class ExtendedKalmanFilter:
    def __init__(self, state, covariance, transition_function, observation_function, transition_covariance, observation_covariance):
        self.state = state
        self.covariance = covariance
        self.transition_function = transition_function
        self.observation_function = observation_function
        self.transition_covariance = transition_covariance
        self.observation_covariance = observation_covariance

    def predict(self):
        self.state = self.transition_function(self.state)
        self.covariance = self.transition_function(self.covariance) + self.transition_covariance

    def update(self, observation):
        residual = observation - self.observation_function(self.state)
        residual_covariance = self.observation_function(self.covariance) + self.observation_covariance
        kalman_gain = self.covariance * self.observation_function(residual_covariance)
        self.state = self.state + kalman_gain * residual
        self.covariance = self.covariance - kalman_gain * self.observation_function(self.covariance)



def ekf(x, P, u, z, F, H, Q, R, f=None, h=None):
    # Predict
    x = f(x, u) if f is not None else F @ x + u
    P = F @ P @ F.T + Q

    # Update
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ (z - h(x) if h is not None else H @ x)
    P = (np.eye(len(x)) - K @ H) @ P

    return x, P

# Initialize state and covariance matrices
x = np.zeros((n, 1))
P = np.eye(n)

# Set state transition matrix, control input matrix, and measurement matrix
F = np.eye(n)
u = np.zeros((n, 1))
H = np.eye(n)

# Set covariance matrices for process and measurement noise
Q = np.eye(n)
R = np.eye(n)

# Perform Kalman filter steps
for i in range(T):
    x, P = ekf(x, P, u, z[i], F, H, Q, R, f=f, h=h)


import numpy as np

class EKF:
    def __init__(self, n, m, f=None, h=None):
        self.n = n  # Number of state variables
        self.m = m  # Number of measurement variables
        self.f = f  # Nonlinear state transition function
        self.h = h  # Nonlinear measurement function

    def predict(self, x, P, u, F, Q):
        x = self.f(x, u) if self.f is not None else F @ x + u
        P = F @ P @ F.T + Q
        return x, P

    def update(self, x, P, z, H, R):
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ (z - self.h(x) if self.h is not None else H @ x)
        P = (np.eye(self.n) - K @ H) @ P
        return x, P

# Define nonlinear state transition and measurement functions
def f(x, u):
    x, y, theta = x[0], x[1], x[2]
    v, w = u[0], u[1]
    return np.array([[x + v * np.cos(theta)],
                     [y + v * np.sin(theta)],
                     [theta + w]])

def h(x):
    x, y, theta = x[0], x[1], x[2]
    return np.array([[x],
                     [y],
                     [theta]])

# Initialize EKF instance
ekf = EKF(3, 3, f=f, h=h)

# Initialize state and covariance matrices