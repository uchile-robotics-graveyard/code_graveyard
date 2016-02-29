#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import math
import smach
import smach_ros
import cv2

from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *
# from cv_bridge import CvBridge, CvBridgeError

#  - - - - macros - - - -

from bender_macros.speech import TalkState
from bender_macros.vision import FindFace_RGBD
from bender_macros.vision import EnrollFace_RGBD
from bender_macros.vision import DetectGender

from bender_core import benpy

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            output_keys=['person_gender','angle_rgbd'])
    
    
    # - - -  parameters - - -
    # pre-enroll speech
    sm.userdata.wait_text = [
         "i dont't see anyone, can you step in front of me?",
         "can you step closer, please?"]
    sm.userdata.wait_timeout = -1  # inf wait
    
    # # enroll images to save
    sm.userdata.n_enroll_images = 10
    sm.userdata.angle_rgbd = 10

    sm.userdata.person_gender = "male" 

    # Fill Machine
    with sm:

        smach.StateMachine.add('WAIT_FOR_OPERATOR', FindFace_RGBD.getInstance(),
                transitions = {'Face Detected':'ENROLL_FACE',
                               'Face Not Detected':'WAIT_FOR_OPERATOR'},
                remapping={'time':'wait_timeout',
                           'textintro':'wait_text',
                           'current_rgbd_angle':'angle_rgbd'}
        )        
        smach.StateMachine.add('ENROLL_FACE', EnrollFace_RGBD.getInstance(),
                transitions = {'succeeded':'GENDER_RECOGNIZER'},
                remapping   = {'n_enroll_images':'n_enroll_images',
                               'angle_rgbd':'angle_rgbd'}
        )
        
        smach.StateMachine.add('GENDER_RECOGNIZER', DetectGender.getInstance(),
                transitions = {'succeeded':'succeeded',
                               'aborted':'succeeded'},
                remapping = {'person_gender':'person_gender'} 
        )
        
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('training_person')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('training_person', sm, '/TRAINING_PERSON_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
