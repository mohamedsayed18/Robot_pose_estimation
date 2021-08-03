#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo(data.linear_acceleration.x)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, callback)    # Todo handle the rate of callback

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()