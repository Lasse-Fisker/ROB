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
    #xy = find_brick_centers()
    mirrorCube([20, -20])

    #RobotDo(invkin([10.6, -10, 10]),0)
    #time.sleep(10)
    #RobotDo(invkin([0, 0, 54.1]),1)    


if __name__ == "__main__":
	top = invkin([0,0, 54.1])          
	RobotDo(top,0)
	main()



