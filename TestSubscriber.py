#!/usr/bin/env python

import rospy
import math
import time

rospy.init_node('GetCoordinates', anonymous=True)

rospy.Subscriber('Coordinates', String, callback)

print callback

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()