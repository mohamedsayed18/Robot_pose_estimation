"""
Kalman filter, This file contain the class of the Kalman filter
"""
import numpy as np

from filterpy.kalman import KalmanFilter

class kalman_matrix:
      def __init__(self):
            self.robot_filter = KalmanFilter(dim_x=9, dim_z=6)    # Filter object
            self.dt = .2      # rate of messages(sensor readings)
            self.x = np.zeros((9,1))   # 9x1 matrix assuming everything is zero
            
            # 6X9 matrix, measurement transation matrix
            self.h = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 1, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 0, 0, 0, 1]])
                              
            # dim=6x6, orientation_covariance and linear_acceleration_covariance
            self.R = np.array([[0.0015387262937311438, 0, 0, 0, 0, 0],
                               [0, 0.0015387262937311438, 0, 0, 0, 0],
                               [0, 0, 0.0015387262937311438, 0, 0, 0],
                               [0, 0, 0, 0.002741552146694444, 0, 0],
                               [0, 0, 0, 0, 0.002741552146694444, 0],
                               [0, 0, 0, 0, 0, 0.007615422629706791]])

            # 9x9 state variance matrix
            self.p = np.diag([50, 50, 50, 100, 100, 100, 5, 5, 5])

            # 9x9 process noise matrix, set to zero for simplicity 
            self.q = np.zeros((9, 9))

      def create_filter(self):
            self.robot_filter.x = self.x
            self.robot_filter.F = np.identity(9)
            self.robot_filter.F[0,3] = 0.5 * self.dt**2
            self.robot_filter.F[1,4] = 0.5 * self.dt**2
            self.robot_filter.F[2,5] = 0.5 * self.dt**2
            
            self.robot_filter.H[:] = self.h
            self.robot_filter.R[:]= self.R
            self.robot_filter.P[:] = self.p
            self.robot_filter.Q[:]= self.q
