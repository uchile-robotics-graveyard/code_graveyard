#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from std_srvs.srv import Empty
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_macros.nav import CheckReach 
from bender_macros.head import MoveAsus

def getInstance():
        
    sm = smach.StateMachine(
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['angle'])

    with sm:
        
        @smach.cb_interface(input_keys=['angle'])
        def goal_request_cb(userdata, request):
            req = NavGoalRequest()
            req.rotation = userdata.angle
            req.goal.header.stamp = rospy.Time.now()            
            return req
        
        smach.StateMachine.add('MOVE_ASUS_NAV',MoveAsus.getNavigationInstance(),
                transitions={'succeeded':'SET_ROTATE_GOAL',
                             'aborted':'SET_ROTATE_GOAL'}
        )
        
        smach.StateMachine.add('SET_ROTATE_GOAL',
            smach_ros.ServiceState('/bender/nav/goal_server/rotate',
                NavGoal,
                request_cb = goal_request_cb,
                input_keys = ['angle']
            ),
            transitions = {
                'succeeded':'CHECK_REACH',
                'aborted':'SET_ROTATE_GOAL'
            },
            remapping = {'angle':'angle'}
        )
        
        smach.StateMachine.add('CHECK_REACH', CheckReach.CheckReach(timeout=30),
                transitions = {
                        'reached':'succeeded',
                        'not_reached':'aborted',
                        'aborted':'aborted',
                        'preempted':'CANCEL_GOAL'
                }
        )

        smach.StateMachine.add('CANCEL_GOAL',
            smach_ros.ServiceState('/bender/nav/goal_server/cancel', Empty),
            transitions = { 'succeeded':'aborted'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('person_recognition')

    sm = getInstance()
    ud = smach.UserData()
    ud.angle = 180

    # introspection server
    sis = smach_ros.IntrospectionServer('rotate_robot', sm, '/ROTATE_ROBOT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
