#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from bender_srvs.srv import *
from bender_msgs.msg import *

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
      #  self.rgbd_position = rospy.ServiceProxy('bender/arm_control/rgbd_position', HeadPosition)


    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        #req2 = HeadPositionRequest()
        #req2.position = 0.15

        #try:
         #   self.rgbd_position(req2)

        #except rospy.ServiceException, e:
         #   print "Service call failed: %s"%e
          #  return 'preempted'


        return 'succeeded'

# define state Position
class Position(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','canceled'],
                             input_keys=['detect_orientation','detect_counter_in'],
                             output_keys=['detect_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Position')
        
        userdata.detect_counter_out = userdata.detect_counter_in + 1
        print "detect_counter_in: ", userdata.detect_counter_in

        if userdata.detect_counter_in == 4:
            return 'canceled'

        try:
            req = NavGoalRequest()
            req.rotation = userdata.detect_orientation[userdata.detect_counter_in]

            rotate_client = rospy.ServiceProxy('/bender/nav/goal_server/rotate', NavGoal)
            rotate_client.wait_for_service()

            resp = rotate_client(req)
            rospy.sleep(3)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'



def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','canceled'],
                            output_keys=['object_position','object_id'])
    sm.userdata.object_position = []
    sm.userdata.object_id = []
    sm.userdata.object_type = []
    sm.userdata.object_status = []
    sm.userdata.detect_orientation = [0,-15,30,-15]
    sm.userdata.detect_counter = 0

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'DETECT'})

        smach.StateMachine.add('DETECT',Detect.getInstance(),
                           transitions={'succeeded':'succeeded',
                                        'notDetected':'DETECT'},
                           remapping={'object_position':'object_position',
                                      'object_id':'object_id'})

        smach.StateMachine.add('POSITION',Position(),
                           transitions={'succeeded':'DETECT'},
                           remapping={'detect_orientation':'detect_orientation',
                                      'detect_counter_in':'detect_counter',
                                      'detect_counter_out':'detect_counter'})

    return sm
 
 
# main
if __name__ == '__main__':

    rospy.init_node('PositionandDetect')

    sm = getInstance()
    ud = smach.UserData()
    ud.any_object = 0;
    # introspection server
    sis = smach_ros.IntrospectionServer('PositionandDetect', sm, '/POSITION_DETECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
