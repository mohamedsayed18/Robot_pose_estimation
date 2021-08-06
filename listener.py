#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from kal import kalman_matrix
from tf import transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

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
    print("orientation_q", orientation_q)
    euler_angles = euler_from_quaternion(orientation_q)
    print("euler angles", euler_angles)
    reading = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                    euler_angles[0], euler_angles[1], euler_angles[2]])
    
    km.robot_filter.predict()
    km.robot_filter.update(reading)
    #data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    print(km.robot_filter.x)
    #move(km.robot_filter.x)
    """
    trans_message = TransformStamped()
    trans_message.header.stamp = data.header.stamp
    trans_message.header.frame_id = data.header.frame_id

    trans_message.transform.translation.x = km.robot_filter.x[0]
    trans_message.transform.translation.y = km.robot_filter.x[1]
    trans_message.transform.translation.y = km.robot_filter.x[2]

    trans_message.transform.rotation.x = 0
    trans_message.transform.rotation.y = 0
    trans_message.transform.rotation.z = 0
    trans_message.transform.rotation.w = 1
    """
    br = TransformBroadcaster()
    quat = quaternion_from_euler(*km.robot_filter.x[6:])
    quat = quat/np.linalg.norm(quat)
    no_rot = (0, 0, 0, 1)
#(km.robot_filter.x[6], km.robot_filter.x[7], km.robot_filter.x[8], km.robot_filter.x[9])
    br.sendTransform(km.robot_filter.x[0:3], quat, data.header.stamp, "bobcat_base", "World")
"""
def move(states):
    #pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    #rate = rospy.Rate(10) # 10hz

    # Create broadcaster
    br = TransformBroadcaster()
    br.
    br.sendTransform((states[0], states[1], states[2]), 
    (0, 0, 0, 1), rospy.Time.now(), "bobcat_base", "world")


    while not rospy.is_shutdown():
        br.sendTransform((2, 2, 0), (0,0,0,1), rospy.Time.now(), "bobcat_base", "world")
        rate.sleep()
    
    
    # The message to sent 
    movecommand = PoseStamped()
    movecommand.pose.position.x = 2
    movecommand.pose.position.y = 2
    movecommand.pose.position.z = 0
     
    while not rospy.is_shutdown():
        pub.publish(movecommand)
        rate.sleep()
"""

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, callback)    # TODO handle the rate of callback

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #rospy.init_node('Moverobot', anonymous=True)
    listener()
