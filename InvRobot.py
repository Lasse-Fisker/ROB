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

def invkin(xyz):
    """
    Python implementation of the the inverse kinematics for the crustcrawler
    Input: xyz position
    Output: Angels for each joint: q1,q2,q3,q4,q5
    
    You might adjust parameters (d1,a1,a2,d4).
    The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
    """

    d1 = 16.8; # cm (height of 2nd joint)
    a1 = 0.0; # (distance along "y-axis" to 2nd joint)
    a2 = 17.31; # (distance between 2nd and 3rd joints)
    d4 = 20; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)
    
    x1 = xyz[0]
    y1 = xyz[1]
    z1 = xyz[2]

    q1 = math.atan2(y1, x1)

    # Calculate q2 and q3
    r2 = math.pow((x1 - a1 * math.cos(q1)),2) + math.pow((y1 - a1 * math.sin(q1)),2)
    s = (z1 - d1)
    D = (r2 + math.pow(s,2) - math.pow(a2,2) - math.pow(d4,2)) / (2 * a2 * d4)
    
    q3 = math.atan2(-math.sqrt(1 - math.pow(D,2)), D)
    q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(q3), a2 + d4 * math.cos(q3))-(math.pi/2)

    #q4 = 0# not consider rotation so far..
    
    #print '______________'
    #print 'r2 = ' , r2
    #print 's = ' , s
    #print 'D = ' , D
    #print 'q1 = ', q1, '::' , ' q2 = ' , q2, '::' , ' q3 = ' , q3
    #print '______________'

    return q1,q2,q3

class RobotExecute:

    N_JOINTS = 5
    def __init__(self,server_name, angles, rotate, gripper):
        self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

        self.joint_positions = []
        self.names =["joint1",
                        "joint2",
                        "joint3",
                        "joint4",
                        "gripper"
                        ]
        # the list of xyz points we want to plan
        joint_positions = [
        [angles[0], angles[1], angles[2], rotate, gripper]
        ]
        # initial duration
        dur = rospy.Duration(1)

        # construct a list of joint positions by calling invkin for each xyz point
        for p in joint_positions:
            jtp = JointTrajectoryPoint(positions=p,velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
            dur += rospy.Duration(5)
            self.joint_positions.append(jtp)

        self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
        self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

    def send_command(self):
        self.client.wait_for_server()
        #print self.goal
        self.client.send_goal(self.goal)

        self.client.wait_for_result()
        #print self.client.get_result()


def RobotDo(angles, rotate, gripper):
        
    print 'RobotExecute started: '
    print '______________________'
    
    node = RobotExecute("/arm_controller/follow_joint_trajectory", angles, rotate, gripper)
    
    node.send_command()

def mirrorCube(xy):

    air1 = 15
    air2 = 30
    table = 6
    Grapped = 0.8
    NotGrapped = 0
    NotRotated = 0
    Rotated = 1.5
    top = invkin([0, 0, 54.1])
    time1 = 1
    time2 = 0.5

    RobotDo(invkin([xy[0], xy[1], air2]), NotRotated, NotGrapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], xy[1], table]), NotRotated, NotGrapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], xy[1], table]), NotRotated, Grapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], xy[1], air1]), Rotated, Grapped)
    time.sleep(time2)
    RobotDo(invkin([xy[0], xy[1], air2]), Rotated, Grapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], -xy[1], air2]), Rotated, Grapped)
    time.sleep(time2)
    RobotDo(invkin([xy[0], -xy[1], air1]), Rotated, Grapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], -xy[1], table]), NotRotated, Grapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], -xy[1], table]), NotRotated, NotGrapped)
    time.sleep(time1)
    RobotDo(invkin([xy[0], -xy[1], air1]), NotRotated, NotGrapped)
    time.sleep(time2)
    RobotDo(invkin([xy[0], -xy[1], air2]), NotRotated, NotGrapped)
    RobotDo(top, NotRotated, NotGrapped)
    time.sleep(2)

def RobotLowFive():

    x = 28
    y = 0
    z = 40
    NotGrapped = 0
    NotRotated = 0
    Rotated = 1.5
    top = invkin([0, 0, 54.1])
    i = 0    

    dur = rospy.Duration(1)

    RobotDo(top, Rotated, NotGrapped)
    time.sleep(0.5)
    RobotDo(top, NotRotated, NotGrapped)

    while i < 31:
        RobotDo(invkin([x, y, z-i]), NotRotated, NotGrapped)
        i += 10

    RobotDo(top, NotRotated, NotGrapped)