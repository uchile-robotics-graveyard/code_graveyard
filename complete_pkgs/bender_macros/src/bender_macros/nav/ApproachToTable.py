#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_srvs.srv import TableDetector
from bender_srvs.srv import TableDetectorResponse
from bender_srvs.srv import Onoff
from geometry_msgs.msg import Twist

# --- macros ---
from bender_macros.nav import ApproachToPlane
from bender_macros.nav import GoToPoseStamped
from bender_macros.nav import GoalFromPlace


def getInstance():

    go_machine =  GoalFromPlace.getMachineInstance( GoToPoseStamped.getInstance() )

    sm = smach.StateMachine(
        outcomes = ['succeeded','aborted','preempted'],
        input_keys = ['table_name','map_name','desired_distance']
    )

    with sm:

        smach.StateMachine.add('GO_TO_TABLE', go_machine,
            transitions = {'succeeded':'GET_CLOSER',
                           'aborted':'GO_TO_TABLE'},
            remapping = {'map_name':'map_name',
                         'place_name':'table_name'}
        )

        smach.StateMachine.add('GET_CLOSER', ApproachToPlane.getInstance(),
            remapping = {'distance':'desired_distance'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('approach_to_place')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.table_name = 'kitchen-pre-tableA'
    ud.map_name = 'map.sem_map'

    # introspection server
    sis = smach_ros.IntrospectionServer('approach_to_table', sm, '/APPROACH_TO_TABLE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
