"""
Kalman filter
"""
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.kalman import update
from filterpy.kalman import predict
from filterpy.common import Q_discrete_white_noise

# Initial values
dt = 1
x = np.array([0, 0, 0, 0, 0, 0]);   # will be the /initial pose
f = np.identity(6);
# 6*6 matrix
h = np.array([[2/dt**2, 0, 0, 0, 0, 0],
              [0, 2/dt**2, 0, 0, 0, 0],
              [0, 0, 2/dt**2, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])
#R = np.zeros((6,6))  # dim=6x6, orientation_covariance and linear_acceleration_covariance
R = np.array([[100, 0, 0, 0, 1, 0],
              [0, 100, 0, 0, 1, 0],
              [0, 0, 100, 0, 1, 0],
              [0, 0, 0, 5, 0, 0],
              [0, 0, 0, 0, 5, 0],
              [0, 0, 0, 0, 0, 5]
              ])

z= np.array([0, 0, 0, 0, 0, 0])

p = np.array([[100, 0, 0, 0, 1, 0],
              [0, 100, 0, 0, 1, 0],
              [0, 0, 100, 0, 1, 0],
              [0, 0, 0, 5, 0, 0],
              [0, 0, 0, 0, 5, 0],
              [0, 0, 0, 0, 0, 5]
              ])

Q = Q_discrete_white_noise(6)

robot_filter = KalmanFilter(dim_x=6, dim_z=6)
robot_filter.x = x
robot_filter.F = f
robot_filter.H = h
robot_filter.R *= R
robot_filter.P[:] = p
robot_filter.Q = Q

v, _ = update(x, p, z, R, h)
print(v)