#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_behaviors.Stage1.BasicFunctionalities import AvoidThat
from bender_behaviors.Stage1.BasicFunctionalities import PickAndPlace
from bender_behaviors.Stage1.BasicFunctionalities import WhatDidYouSay
from bender_macros.speech import WaitCommand
from bender_macros.nav import GoToPlace
from bender_macros.vision import WaitOpenDoor


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.map_name = 'map.sem_map'

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    sm.userdata.avoid_that_place_name = 'wdys'
    sm.userdata.pre_avoid_that_place_name = 'avoid_that'
    sm.userdata.wdys_place = 'kitchen'
    sm.userdata.pick_and_place_pick_place = 'kitchen_table'
    sm.userdata.exit_place = 'door_right_post_out'
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    with sm:

        smach.StateMachine.add('WAIT_FOR_OPEN_DOOR',WaitOpenDoor.getInstance(),
                transitions = {'succeeded':'PICK_AND_PLACE'}
        )
        smach.StateMachine.add('PICK_AND_PLACE',PickAndPlace.getInstance(),
                transitions={'succeeded':'GO_TO_PRE_AVOID_THAT'},
                remapping={'map_name':'map_name',
                           'pick_place':'pick_and_place_pick_place'
                }
        )
        smach.StateMachine.add('GO_TO_PRE_AVOID_THAT', GoToPlace.getInstance(),
                transitions = {'succeeded':'WAIT_COMMAND1'},
                remapping = {'place_name':'pre_avoid_that_place_name',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('WAIT_COMMAND1',WaitCommand.getInstance(),
                transitions={'succeeded':'AVOID_THAT'}
        )
        smach.StateMachine.add('AVOID_THAT',AvoidThat.getInstance(),
                transitions={'succeeded':'WAIT_COMMAND2'},
                remapping={'place_name':'avoid_that_place_name',
                           'map_name':'map_name'}
        )
        smach.StateMachine.add('WAIT_COMMAND2',WaitCommand.getInstance(),
                transitions={'succeeded':'WHAT_DID_YOU_SAY'}
        )
        smach.StateMachine.add('WHAT_DID_YOU_SAY',WhatDidYouSay.getInstance(),
                transitions={'succeeded':'WAIT_COMMAND_FINAL'}
        )
        smach.StateMachine.add('WAIT_COMMAND_FINAL',WaitCommand.getInstance(),
                transitions={'succeeded':'EXIT'}
        )
        smach.StateMachine.add('EXIT',GoToPlace.getInstance(),
                transitions={'succeeded':'succeeded'},
                remapping={'place_name':'exit_place',
                           'map_name':'map_name'}
        )

        sm.set_initial_state(['WAIT_COMMAND1'])
        #sm.set_initial_state(['WAIT_COMMAND2'])

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('BasicFunctionalitiesFull')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('basic_functionalities', sm, '/BASIC_FUNCTIONALITIES_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()