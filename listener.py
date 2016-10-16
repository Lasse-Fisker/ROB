#!/usr/bin/env python
import rospy
from std_msgs.msg import String

visionCoor = []

def getCoordinates(coord):    

    coord_str = str(coord)
    coor = coord_str.replace("data: ","")
    visionCoor = [float(s) for s in coor.split(',')]
    