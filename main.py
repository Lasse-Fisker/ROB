#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math
import time
from std_msgs.msg import String

from InvRobot import *
from findBricks import *
#from listener import *

visionCoor = []

def getCoordinates(coord):    
    global visionCoor
    
    coord_str = str(coord)
    coor = coord_str.replace("data: ","")
    visionCoor = [float(s) for s in coor.split(',')]
    
    if visionCoor[0] == 0 and visionCoor[1] == 0:
        print "Nu skal programmet stoppe"
        visionCoor = []
        print "VisionCoor"
        print visionCoor

def main():
    Jobactive = True
    Job = False
    
    while Jobactive == True:
        print "V3"
        print visionCoor
        xy = visionCoor
        if len(xy) > 0:
            mirrorCube(xy)
            Job = True
        else:
            Jobactive = False
            if Job == True:
                RobotLowFive()
                print "Jobs done"
            else:
                print "There wasnt any job "
                
if __name__ == "__main__":
    rospy.init_node("InvRobot")
    rospy.Subscriber("Coordinates", String, getCoordinates)
    setupGrabberPressureSensor()

    top = invkin([0,0, 54.1])
    RobotDo(top,0,0)
    main()



