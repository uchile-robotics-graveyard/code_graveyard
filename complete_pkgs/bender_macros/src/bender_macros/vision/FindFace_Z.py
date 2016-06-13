#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import cv2

from bender_core import benpy

#  - - - - macros - - - -
from bender_macros.speech import Talk
from bender_macros.head import FaceOrder
import sensor_msgs
import FindFace_RBGD

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class 

def getInstance():


    sm = smach.StateMachine(outcomes=['Face Detected','Face Not Detected','succeeded','aborted','preempted'])

    sm.userdata.enroll_face_id = -1
    sm.userdata.sm_main_face = Image()
    sm.userdata.image = Image()

    sm.userdata.angle_face = [-10,0,25,35]
    sm.userdata.angle_facerecognition = 0

    rgbd_angles = [-10,0,25,35]
    sm.userdata.current_rgbd_angle = 0

    with sm:
        smach.StateMachine.add('FIND_PITCH',FindFace_RBGD,
                transitions={'succeeded':''}
        )
        smach.StateMachine.add('DETECT_FACES',DetectFaces(),
                           transitions={'Face Detected': 'Face Detected', 
                           'Face Not Detected': 'SELECT_RGBD_ANGLE', 
                                    'succeeded':'DETECT_FACES',
                                    'aborted':'DETECT_FACES'},
                           remapping={'main_face':'sm_main_face'})
        ## scan looking for a shelf
        smach.StateMachine.add('SELECT_RGBD_ANGLE', SelectRGBDAngle(rgbd_angles),
            transitions={'succeeded':'MOVE_ASUS',
                         'fail':'Face Not Detected'},
            remapping={'selected':'current_rgbd_angle'}
        )
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('find_face_rgbd')

    sm = getInstance()
    ud = smach.UserData()

    # introspection server
    sis = smach_ros.IntrospectionServer('find_face_rgbd', sm, '/FIND_FACE_RGBD_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
