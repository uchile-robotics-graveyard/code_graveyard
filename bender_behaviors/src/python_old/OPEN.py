#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import math
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *

import std_srvs
from locale import str
from entrance import button_state
from std_msgs.msg._Float64 import Float64
    
speech_talking=False
speech_returned=False
speech_talking=False
speech_bad_detection=False
    
def talk(text):
    global speech_talking
    speechSynth= rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
    speechSynth(text)
    speech_talking = False
    while not speech_talking:
        Sleep(0.5)
    while speech_talking:
        Sleep(0.5)

def speechSynthStatusCallback(data):
    global speech_talking
    speech_talking=(data.data=='Talking')

def bubblesort(A,B):
     for i in range( len( A ) ):
         for k in range( len( A ) - 1, i, -1 ):
             if ( A[k] < A[k - 1] ):
                 swap( A, k, k - 1 )
                 swap( B, k, k - 1 )

def swap( A, x, y ):
     tmp = A[x]
     A[x] = A[y]
     A[y] = tmp
 

def main():

    
    rospy.init_node('entrance')
    
    reset_srv = rospy.ServiceProxy('/arm_vision_interface/reset_obj', Dummy)
    detect_srv = rospy.ServiceProxy('/arm_vision_interface/detect_obj', ObjectDetection)
    rospy.wait_for_service('/arm_vision_interface/reset_obj')
    rospy.wait_for_service('/arm_vision_interface/detect_obj')
    speechSynthState=rospy.Subscriber('/speech_synthesizer/status',String,speechSynthStatusCallback)    

    print "working"
    
    od = ObjectDetection()
    
    
    while True:
        
        # Reseteo
        reset_srv()
        Sleep(3)
        response = detect_srv()
        detection = response.name
        y_det = response.y
        
        bubblesort(y_det, detection)
        
        talk("I recognize "+ str(len(y_det)) + " objects")
        
        cnt = 0
        talk("from right to left")
        while cnt<len(y_det):
            talk(response.name[cnt])
            cnt+=1    
        
if __name__ == '__main__':
    main()