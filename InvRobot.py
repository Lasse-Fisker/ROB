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
    Output: Angels for each joint: q1,q2,q3,q4
    
    You might adjust parameters (d1,a1,a2,d4).
    The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
    """

    d1 = 16.8; # cm (height of 2nd joint)
    a1 = 0.0; # (distance along "y-axis" to 2nd joint)
    a2 = 17.31; # (distance between 2nd and 3rd joints)
    d4 = 23.5; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

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

    q4 = 0 # not consider rotation so far..
    
    print '______________'
    print 'r2 = ' , r2
    print 's = ' , s
    print 'D = ' , D
    print 'q1 = ', q1, '::' , ' q2 = ' , q2, '::' , ' q3 = ' , q3
    print '______________'


    return q1,q2,q3,q4

class RobotExecute:

    N_JOINTS = 5
    def __init__(self,server_name, angles, gripper):
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
        [angles[0], angles[1], angles[2], 0, gripper]
        ]
        # initial duration
        dur = rospy.Duration(2)

        # construct a list of joint positions by calling invkin for each xyz point
        for p in joint_positions:
            jtp = JointTrajectoryPoint(positions=p,velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
            dur += rospy.Duration(2)
            self.joint_positions.append(jtp)

        self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
        self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

    def send_command(self):
        self.client.wait_for_server()
        print self.goal
        self.client.send_goal(self.goal)

        self.client.wait_for_result()
        print self.client.get_result()


def RobotDo(angles, gripper):
    
    rospy.init_node("au_dynamixel_test_node")
    
    node= RobotExecute("/arm_controller/follow_joint_trajectory", angles, gripper)

    node.send_command()

def mirrorCube(xy);

    air = 30
    table = 4
    Grapped = 0.7
    NotGrapped = 0
    top = invkin([0, 0, 54])

    RobotDo(invkin([xy[0], xy[1], air]), NotGrapped)
    time.sleep(2)
    RobotDo(invkin([xy[0], xy[1], table]), NotGrapped)
    times.leep(2)
    RobotDo(invkin([xy[0], xy[1], table]), Grapped)
    time.sleep(2)
    RobotDo(invkin([xy[0], xy[1], air]), Grapped)
    time.sleep(2)
    RobotDo(invkin([xy[0], -xy[1], air]), Grapped)
    time.sleep(2)
    RobotDo(invkin(xy[0], -xy[1], air], Grapped)
    time.sleep(2)
    RobotDo(invkin(xy[0], -xy[1], table]), NotGrapped)
    time.sleep(2)
    RoboDo(top, NotGrapped)
    time.sleep(2)
