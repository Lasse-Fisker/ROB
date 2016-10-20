#!/usr/bin/env python
import rospy
from std_msgs.msg import BooleanType

# Callback function for grabbed something topic - val is wheather or not the grabber is holding something.
def grabbedSomethingCallback(val) {
    if val != null and type(val) == types.BooleanType 
        return val
}

# toggle wheather or not the grabber should check for pressure
def toggleGrabberSensor(shouldSense) {
    pub = rospy.Publisher("grabber_check_pressure", BooleanType, queue_size=10)
    rospy.init_node('strainGaugeToggle', anonymous=True)
    pub.publish(shouldSense)
}