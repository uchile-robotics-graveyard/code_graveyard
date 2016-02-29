#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from bender_macros.nav import GoToPlace

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys=['map_name','place_name'])

    with sm:

        smach.StateMachine.add('GO_TO_PLACE', GoToPlace.getInstance(),
                transitions = {'succeeded':'succeeded'},
                remapping = {'place_name':'place_name',
                             'map_name':'map_name'}
        )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('avoid_that')

    sm = getInstance()

    ud = smach.UserData()
    ud.map_name = 'map.sem_map'
    ud.place_name = 'wdys'

    # introspection server
    sis = smach_ros.IntrospectionServer('avoid_that', sm, '/AVOID_THAT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
