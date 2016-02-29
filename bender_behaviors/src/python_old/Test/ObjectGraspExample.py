#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.arm import GraspObject

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        rospy.sleep(0.1)
        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','notGrabbed'])
    
    sm.userdata.position = [65,10,81]

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'GRASP_OBJECT'})

        smach.StateMachine.add('GRASP_OBJECT',GraspObject.getInstance(),
                           remapping={'position':'position'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('GraspObjectExample')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('grasp_object_example', sm, '/GRASP_OBJECT_EXAMPLE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
