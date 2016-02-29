#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.vision import PositionAndDetect

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

# define state DisplayInfov
class DisplayInfo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['object_position','object_id','object_type','object_status'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DisplayInfo')
        print "object position x: ", userdata.object_position['x']
        print "object_position y: ", userdata.object_position['y']
        print "object_position z: ", userdata.object_position['z']
        print "object id: ", userdata.object_id
        print "object type: ", userdata.object_type
        print "object status: ", userdata.object_status
        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','canceled'],
                            output_keys=['object_position','object_id'])
    sm.userdata.object_position = {}
    sm.userdata.object_id = []
    sm.userdata.object_type = []
    sm.userdata.object_status = []

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'POSITION_AND_DETECT'})

        smach.StateMachine.add('POSITION_AND_DETECT',PositionAndDetect.getInstance(),
                           transitions={'succeeded':'DISPLAY_INFO'},
                           remapping={'object_position':'object_position',
                                      'object_id':'object_id',
                                      'object_type':'object_type',
                                      'object_status':'object_status'})

        smach.StateMachine.add('DISPLAY_INFO',DisplayInfo(),
                           remapping={'object_position':'object_position',
                                      'object_id':'object_id',
                                      'object_type':'object_type',
                                      'object_status':'object_status'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('ObjectDetectionExample')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('object_detection_example', sm, '/OBJECT_DETECTION_EXAMPLE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
