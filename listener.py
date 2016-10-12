#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def getCoordinates(coord):    

    coord_str = str(coord)
    coor = coord_str.replace("data: ","")
    xy = [float(s) for s in coor.split(',')]
    return xy
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("Coordinates", String, getCoordinates)

    rospy.spin()