#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from SmachTutorial import SubMachine 

class Foo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'])
        
    def execute(self, userdata):

        rospy.sleep(1.0)
        
        return 'succeeded'

    def request_preempt(self):
        # this method should not block longer than it takes to notify 
        # any child threads / processes to preempt
        smach.State.request_preempt(self)


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with sm:
        
        smach.StateMachine.add('STATE_1',Foo(),
            transitions = {'succeeded':'STATE_2'}
        )
        
        smach.StateMachine.add('STATE_2',Foo(),
            transitions = {'succeeded':'BAR'}
        )

        smach.StateMachine.add('BAR', SubMachine.getInstance(),
            transitions = {'succeeded':'STATE_3'}
        )
        
        smach.StateMachine.add('STATE_3',Foo(),
            transitions = {'succeeded':'succeeded'}
        )
        
        initial_data = smach.UserData()
        sm.set_initial_state(['STATE_2'],initial_data)

    return sm

   

# main
if __name__ == '__main__':

    rospy.init_node('machine')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('machine', sm, '/MACHINE_SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


