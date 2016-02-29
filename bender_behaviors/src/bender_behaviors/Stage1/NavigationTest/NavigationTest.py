#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

#  - - - - macros - - - -
from bender_macros.vision import WaitOpenDoor
from bender_macros.nav    import RoomEntering
from bender_macros.nav    import InteractiveGoToPlace
from bender_macros.nav    import GoToPlace
from bender_macros.speech import TalkState
from bender_macros.interaction import FollowMe

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # machine parameters
    sm.userdata.map_name   = 'map.sem_map'
    sm.userdata.waypoint_1 = 's1-navtest-waypoint-1'
    sm.userdata.waypoint_2 = 's1-navtest-waypoint-2'
    sm.userdata.waypoint_3 = 's1-navtest-waypoint-3'
    sm.userdata.exit_pre   = 'door_B_exit_pre'
    sm.userdata.exit_post  = 'door_B_exit_post'

    sm_wait_door   = WaitOpenDoor.getInstance()
    sm_int_go_to_place = InteractiveGoToPlace.getInstance()
    sm_go_to_place = GoToPlace.getInstance()

    with sm:

        smach.StateMachine.add('WAIT_FOR_OPEN_DOOR', sm_wait_door,
                transitions = {'succeeded':'ROOM_ENTERING'}
        )

        smach.StateMachine.add('ROOM_ENTERING',RoomEntering.Entrance(),
                transitions = {'succeeded':'ANOUNCE_START'},
        )

        smach.StateMachine.add('ANOUNCE_START', 
                TalkState.getInstance(text='i am going to the first waypoint'),
                transitions = {'succeeded':'GO_TO_WAYPOINT_1'},
        )

        smach.StateMachine.add('GO_TO_WAYPOINT_1', sm_int_go_to_place,
                transitions = {'succeeded':'ANOUNCE_SUCCEEDED_1'},
                remapping = {'place_name':'waypoint_1',
                             'map_name':'map_name'}
        )

        smach.StateMachine.add('ANOUNCE_SUCCEEDED_1', 
                TalkState.getInstance(text='now i am going to the second waypoint'),
                transitions = {'succeeded':'GO_TO_WAYPOINT_2'},
        )

        smach.StateMachine.add('GO_TO_WAYPOINT_2', sm_int_go_to_place,
                transitions = {'succeeded':'ANOUNCE_SUCCEEDED_2'},
                remapping = {'place_name':'waypoint_2',
                             'map_name':'map_name'}
        )

        smach.StateMachine.add('ANOUNCE_SUCCEEDED_2',
                TalkState.getInstance(text='and now, i will look for someone to follow'),
                transitions = {'succeeded':'GO_TO_WAYPOINT_3'},
        )

        smach.StateMachine.add('GO_TO_WAYPOINT_3', sm_int_go_to_place,
                transitions = {'succeeded':'FOLLOW_SM'},
                #transitions = {'succeeded':'DUMMY_FOLLOW_SM'},
                remapping = {'place_name':'waypoint_3',
                             'map_name':'map_name'}
        )
        
        smach.StateMachine.add('FOLLOW_SM',
                FollowMe.getInstance(),
                transitions = {'succeeded':'ANOUNCE_RETURNING_TO_WP3'},
        )

        smach.StateMachine.add('ANOUNCE_RETURNING_TO_WP3',
                TalkState.getInstance(text='finally, i will return to my home'),
                transitions = {'succeeded':'RETURN_TO_WP3'},
        )

        # esto podria ser problema: pues podria replanear en cualquier 
        # momento, e intentar retornar por una puerta distinta
        smach.StateMachine.add('RETURN_TO_WP3', sm_go_to_place,
                transitions = {'succeeded':'ANOUNCE_PRE_EXIT'},
                remapping = {'place_name':'waypoint_3',
                             'map_name':'map_name'}
        )
        
        smach.StateMachine.add('ANOUNCE_PRE_EXIT',
                TalkState.getInstance(text='i am done, now i will leave the arena'),
                transitions = {'succeeded':'GO_TO_PRE_EXIT'},
        )        

        smach.StateMachine.add('GO_TO_PRE_EXIT', sm_go_to_place,
                transitions = {'succeeded':'ANOUNCE_EXIT'},
                remapping = {'place_name':'exit_pre',
                             'map_name':'map_name'}
        )

        smach.StateMachine.add('ANOUNCE_EXIT',
                TalkState.getInstance(text='Bye bye'),
                transitions = {'succeeded':'EXIT'},
        )

        smach.StateMachine.add('EXIT', sm_go_to_place,
                transitions = {'succeeded':'succeeded'},
                remapping = {'place_name':'exit_post',
                             'map_name':'map_name'}
        )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('NavigationTest')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('navigation_test', sm, '/NAVIGATION_TEST_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()

