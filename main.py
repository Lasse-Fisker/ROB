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

from invRobot import *



def main():
    mirrorCube([20, 20])
    
	

if __name__ == "__main__":
	top = invkin([0, 0, 54])
	RobotDo(top,0)
	main()



