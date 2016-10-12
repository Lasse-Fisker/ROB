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

from InvRobot import *
from findBricks import *



def main():
    Jobactive = True
    Job = False 
    while Jobactive == True:
        xy = find_brick_centers()
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
	top = invkin([0,0, 54.1])          
	RobotDo(top,0,0)
	main()



