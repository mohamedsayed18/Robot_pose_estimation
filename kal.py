"""
Kalman filter
"""
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.kalman import predict, update
#from filterpy.kalman import update
#from filterpy.kalman import predict
from filterpy.common import Q_discrete_white_noise


class kalman_matrix:
    def __init__(self):
        self.robot_filter = KalmanFilter(dim_x=6, dim_z=6)
        self.dt = .2
        self.x = np.array([0, 0, 0, 0, 0, 0]);   # 6x1 matrix assuming everything is zero
        self.f = np.identity(6);    
        # 6*6 matrix
        self.h = np.array([[0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        #R = np.zeros((6,6))  # dim=6x6, orientation_covariance and linear_acceleration_covariance
        self.R = np.array([[0.0015387262937311438, 0, 0, 0, 0, 0],
                           [0, 0.0015387262937311438, 0, 0, 0, 0],
                           [0, 0, 0.0015387262937311438, 0, 0, 0],
                           [0, 0, 0, 0.002741552146694444, 0, 0],
                           [0, 0, 0, 0, 0.002741552146694444, 0],
                           [0, 0, 0, 0, 0, 0.007615422629706791]
              ])

        self.z= np.array([9.74722984, 0.09411442, 0.63091934,
                        20, 20, 30])

        self.p = np.diag([50, 50, 50, 5, 5, 5])

        self.q = np.identity(6)
        self.u = np.array([9.74722984, 0.09411442, 0.63091934])

    def create_filter(self):
        
      self.robot_filter.x = self.x
      self.robot_filter.F = self.f
      self.robot_filter.H = self.h
      self.robot_filter.R *= self.R
      self.robot_filter.P[:] = self.p
      self.robot_filter.Q [:]= self.q
      """
      self.robot_filter.B = np.array([[0.5*self.dt**2, 0, 0],
                                    [0, 0.5*self.dt**2, 0],
                                    [0, 0, 0.5*self.dt**2],
                                    [0, 0, 0],
                                    [0, 0, 0],
                                    [0, 0, 0]])
      """
"""
km = kalman_matrix()
km.create_filter()

xs, cov = [], []
for i in range(100):
      km.robot_filter.predict(km.u)
      km.robot_filter.update(km.z)
      xs.append(km.robot_filter.x)
      cov.append(km.robot_filter.P)
      
xs = np.array(xs)
print(xs[:,3:])
"""    
"""
for _ in range(10):
      km.x, km.p = predict(km.x, km.p, km.f, u=km.u, B=km.robot_filter.B)
      print(km.x)


v, _ = update(km.x, km.p, km.z, km.R, km.h)
print(v)
print(km.z)
"""

# Notes
"""
linear_acceleration_covariance: [0.0015387262937311438, 0.0, 0.0, 0.0, 0.0015387262937311438, 0.0, 0.0, 0.0, 0.0015387262937311438]
orientation_covariance: [0.002741552146694444, 0.0, 0.0, 0.0, 0.002741552146694444, 0.0, 0.0, 0.0, 0.007615422629706791]

The first message is at: 8:43:19
time step is 0.2602640190000000131
"""