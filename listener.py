#!/usr/bin/env python3

import numpy as np

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

from kal import kalman_matrix


# Create the kalman filter
km = kalman_matrix()
km.create_filter()

def Imu_callback(data):
    """
    Callback to Imu
    1. get the new measurements
    2. perform update
    """
    # Get the measurements
    orientation_q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    # Change them to euler angles
    euler_angles = euler_from_quaternion(orientation_q)
    # measurement which contain the linear acceleration followed by orientation
    reading = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                    euler_angles[0], euler_angles[1], euler_angles[2]])
    
    km.robot_filter.predict()   # predict, calculate the prior
    km.robot_filter.update(reading) # update, takes the sensor the measurement

    print(km.robot_filter.x)    # the new states

    br = TransformBroadcaster() # broadcast transformation between two frames
    quat = quaternion_from_euler(*km.robot_filter.x[6:])    # Change orientation to quaternion
    quat = quat/np.linalg.norm(quat)    # get the norm due to some computation accuracy 
    
    # send the transformation
    br.sendTransform(km.robot_filter.x[0:3], quat, data.header.stamp, "bobcat_base", "World")

if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, Imu_callback)
    rospy.spin()
