#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from bender_msgs.msg import Emotion
from bender_srvs.srv import synthesize
from bender_macros.vision import WaitOpenDoor
from bender_macros.nav import RoomEntering
from bender_macros.nav import GoToPlace
from bender_macros.speech import Talk, TalkState
from bender_macros.head import FaceOrder
from bender_macros.skills import PersonActions

from bender_core import benpy

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        
        FaceOrder.ChangeFace(0)
        FaceOrder.ChangeFace("happy1")

        # anounce ready
        text = 'I am ready to start'
        Talk.getInstance(text,len(text)/8)

        return 'succeeded'

class Advertise(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):

        # Put happy face
        FaceOrder.ChangeFace("happy3")
        
        # anounce ready
        PersonActions.Tell_Introduce()

        # Put happy face
        FaceOrder.ChangeFace("happy1")

        return 'succeeded'

class GoodBye(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        
        # Put happy face
        FaceOrder.ChangeFace("happy1")

        # anounce ready
        text = 'bye bye'
        Talk.getInstance(text,len(text)/8)

        return 'succeeded'


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.map_name = 'map.sem_map'
    sm.userdata.place_entrance = 'door_A_enter_post'
    sm.userdata.place_living = 'rips_place'
    sm.userdata.place_exit = 'door_B_exit_post'

    with sm:
        smach.StateMachine.add('SETUP',Setup(),
                transitions = {'succeeded':'WAIT_FOR_OPEN_DOOR'}
        )

        smach.StateMachine.add('WAIT_FOR_OPEN_DOOR', WaitOpenDoor.getInstance(),
                transitions = {'succeeded':'ROOM_ENTERING'}
        )

        smach.StateMachine.add('ROOM_ENTERING',RoomEntering.Entrance(),
                transitions = {'succeeded':'ANOUNCE_START'},
        )
        smach.StateMachine.add('ANOUNCE_START', 
                TalkState.getInstance(text='i am going to the waypoint'),
                transitions = {'succeeded':'GO_TO_ENTRANCE'},
        )
        smach.StateMachine.add('GO_TO_ENTRANCE', GoToPlace.getInstance(),
                transitions = {'succeeded':'GO_TO_LIVING_ROOM'},
                remapping = {'place_name':'place_entrance',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('GO_TO_LIVING_ROOM', GoToPlace.getInstance(),
                transitions = {'succeeded':'ADVERTISE'},
                remapping = {'place_name':'place_living',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('ADVERTISE', Advertise(),
                transitions = {'succeeded':'GO_TO_EXIT'}
        )
        smach.StateMachine.add('GO_TO_EXIT', GoToPlace.getInstance(),
                transitions = {'succeeded':'GOOD_BYE'},
                remapping = {'place_name':'place_exit',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('GOOD_BYE',GoodBye(),
                transitions = {'succeeded':'succeeded'}
        )

        # ud = smach.UserData()
        # sm.set_initial_state(['GO_TO_LIVING_ROOM'],ud)

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('rips')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('rips', sm, '/RIPS_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
