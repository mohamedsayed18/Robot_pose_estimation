#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from kal import kalman_matrix
from tf.transformations import euler_from_quaternion

import numpy as np

# Global variable TODO: Avoid global variables
km = kalman_matrix()
km.create_filter()

def callback(data):
    """
    Callback to Imu
    1. get the new measurements
    2. perform update
    """
    orientation_q = [data.orientation.x, data.orientation.y, data.orientation.y, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_q)
    reading = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                    roll, pitch, yaw])
    
    km.robot_filter.predict()
    km.robot_filter.update(reading)
    #data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    print(km.robot_filter.x[0])
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, callback)    # TODO handle the rate of callback

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()