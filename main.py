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
    print "V1"
    print visionCoor
    


def main():
    Jobactive = True
    Job = False
    rospy.Subscriber("Coordinates", String, getCoordinates)

    print "V2"
    print visionCoor
    
    while Jobactive == True:
        print "V3"
        print visionCoor
        xy = visionCoor
        if xy[0] == 0 and xy[1] == 0:
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
    top = invkin([0,0, 54.1])
    RobotDo(top,0,0)
    main()



