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
from std_msgs.msg import Int32

grabberPressure = 0;

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

    #constructur
    def __init__(self,server_name, angles, rotate, gripper, speed):
        self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

        self.names =["joint1",
                        "joint2",
                        "joint3",
                        "joint4",
                        "gripper"
                        ]
        # the list of xyz points we want to plan
        self.joint_positions = [
        [angles[0], angles[1], angles[2], rotate, gripper]
        ]

        # initial duration
        dur = speed

        # construct a list of joint positions
        for p in joint_positions:
            jtp = JointTrajectoryPoint(positions=p,velocities=[0.5]*self.N_JOINTS, time_from_start=dur)
            self.joint_positions.append(jtp)
        
        # create joint trajectory
        self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)

        # create goal
        self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

    def send_command(self):
        self.client.wait_for_server()
        #print self.goal
        self.client.send_goal(self.goal)

        self.client.wait_for_result()
        #print self.client.get_result()


def RobotDo(angles, rotate, gripper, speed):
        
    print 'RobotExecute started: '
    print '______________________'
    
    node = RobotExecute("/arm_controller/follow_joint_trajectory", angles, rotate, gripper, speed)
    
    node.send_command()

def mirrorCube(xy):

    air1 = 15
    air2 = 30
    table = 6
    grapper_pos = 0.8
    not_rotated = 0
    rotated = 1.5
    top = invkin([0, 0, 54.1])
    time1 = 1
    time2 = 0.5

    #in air
    RobotDo(invkin([xy[0], xy[1], air2]), not_rotated, grapper_pos, 1)
    time.sleep(time1)

    #to table
    RobotDo(invkin([xy[0], xy[1], table]), not_rotated, grapper_pos, 2)
    time.sleep(time1)

    #grab brick. While pressure is low enough, increase grabber position until brick is secured.
    while grabberPressure < 700:
        grapper_pos =+ 0.1
        RobotDo(invkin([xy[0], xy[1], table]), not_rotated, grapper_pos, 1)
        time.sleep(time2)

    #rotate - to air low
    RobotDo(invkin([xy[0], xy[1], air1]), rotated, grapper_pos, 4)
    time.sleep(time2)

    #to air 
    RobotDo(invkin([xy[0], xy[1], air2]), rotated, grapper_pos, 2)
    time.sleep(time1)

    #to mirrored X Y
    RobotDo(invkin([xy[0], -xy[1], air2]), rotated, grapper_pos, 3)
    time.sleep(time2)

    # lower
    RobotDo(invkin([xy[0], -xy[1], air1]), rotated, grapper_pos, 1)
    time.sleep(time1)

    #table un rotate
    RobotDo(invkin([xy[0], -xy[1], table]), not_rotated, grapper_pos, 5)
    time.sleep(time1)

    #release
    grapper_pos = 0
    RobotDo(invkin([xy[0], -xy[1], table]), not_rotated, grapper_pos, 2)
    time.sleep(time1)

    # to air
    RobotDo(invkin([xy[0], -xy[1], air1]), not_rotated, grapper_pos, 1)
    time.sleep(time2)

    # higher air
    RobotDo(invkin([xy[0], -xy[1], air2]), not_rotated, grapper_pos, 6)
    RobotDo(top, not_rotated, grapper_pos)
    time.sleep(2)

def RobotLowFive():

    x = 28
    y = 0
    z = 40
    not_rotated = 0
    rotated = 1.5
    top = invkin([0, 0, 54.1])
    i = 0    

    dur = rospy.Duration(1)

    RobotDo(top, rotated, 0, 2)
    time.sleep(0.5)
    RobotDo(top, not_rotated, 0, 2)

    while i < 31:
        RobotDo(invkin([x, y, z-i]), not_rotated, 0, 2)
        i += 10

    RobotDo(top, not_rotated, 0, 2)

# setup subscriber for pressure sensor
def setupGrabberPressureSensor():
    rospy.Subscriber("grabber_pressure", Int32, handleReadPressure)

# callback method for pressure sensor
def handleReadPressure(val):
    grabberPressure = val

    #debug
    print "grabber pressure is: %d" % val.data
