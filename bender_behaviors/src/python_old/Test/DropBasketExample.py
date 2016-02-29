#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.arm import DropBasket

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

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    sm.userdata.selected_arm_local = '/left_arm'
    sm.userdata.basket_side_local = 'right'

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'DROP_BASKET'})

        smach.StateMachine.add('DROP_BASKET',DropBasket.getInstance(),
                           remapping={'selected_arm':'selected_arm_local',
                                      'basket_side':'basket_side_local'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('DropBasketExample')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('drop_basket_example', sm, '/DROP_BASKET_EXAMPLE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
