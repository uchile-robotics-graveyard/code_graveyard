#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import serial
from std_msgs.msg import Bool
import std_msgs

s = serial.Serial('/dev/ttyACM1', 19200, timeout=1) #se asigna el puerto serial a la variable s

def stopButton():
    rospy.init_node('stop_button')
    pub=rospy.Publisher("/stop_button",Bool)
    while not rospy.is_shutdown():
        c=s.read()
        if c == chr(0):
            pub.publish(False)
        elif c == chr(255):
            pub.publish(True)
        else:
            print "wrong lecture"
        
        #
if __name__ == '__main__':
    stopButton()
