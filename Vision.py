#!/usr/bin/env python

import rospy
from findBricks import *
from std_msgs.msg import String

def VisionPublisher():
    pub = rospy.Publisher('Coordinates', String, queue_size=10)
    rospy.init_node('VisionPublisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        bricks = find_brick_centers()
        while i > 0:
            coords_str = "%f,%f"%(bricks[0], bricks[1])
            
        pub.publish(test_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        VisionPublisher()
    except rospy.ROSInterruptException:
        pass