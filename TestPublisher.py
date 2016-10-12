#!/usr/bin/env python

import rospy
import math
import time

def VisionCoordinates():
    pub = rospy.Publisher('Coordinates', list, queue_size=10)
    rospy.init_node('VisionCoordinates', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        listen = [20,20]
        pub.publish(listen)
        rate.sleep()

if __name__ == '__main__':
    VisionCoordinates()