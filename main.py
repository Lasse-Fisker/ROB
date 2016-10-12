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



def main():
    mirrorCube([12.3, -14.6])
    mirrorCube([5.1, -23.4])
    mirrorCube([24.6, -17.1])
    mirrorCube([17, -26.3])

    #RobotDo(invkin([10.6, -10, 10]),0)
    #time.sleep(10)
    #RobotDo(invkin([0, 0, 54.1]),1)    


if __name__ == "__main__":
	top = invkin([0,0, 54.1])          
	RobotDo(top,0)
	main()



