#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from bender_macros.vision import Detect
from bender_macros.arm import GraspCartesian
from geometry_msgs.msg import PoseStamped
#from bender_utils.ros import benpy

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                              output_keys=['grasped'])

    sm.userdata.arm = 'l_arm'
    p = PoseStamped()
    p.pose.position.x = 0.8
    p.pose.position.y = 0.2
    p.pose.position.z = 0.9
    p.pose.orientation.w = 1
    sm.userdata.pose = p

    with sm:

#        smach.StateMachine.add('DETECT', Detect.getInstance(),
#                           transitions={'succeeded':'GRASP'},
#                           remapping={'arm':'arm','trayectory_name':'trayectory_name_before'}
#                           )

        smach.StateMachine.add('GRASP', GraspCartesian.getInstance(),
                           transitions={'succeeded':'succeeded'},
                           remapping={'arm':'arm','pose':'pose'}
                           )

    sm.set_initial_state(['GRASP'])

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('DetectAndGrasp')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('DetectAndGrasp', sm, '/DETECT_AND_GRASP_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()