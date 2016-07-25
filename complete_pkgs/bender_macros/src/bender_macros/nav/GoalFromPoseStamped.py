#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped 
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_macros.nav import CheckReach 
from bender_macros.head import MoveAsus

def getServiceInstance(service_str):
        
    sm = smach.StateMachine(
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['goal_pose']
    )

    with sm:
        
        @smach.cb_interface(input_keys=['goal'])
        def goal_request_cb(userdata, request):
            req = NavGoalRequest()
            req.goal = userdata.goal
            req.goal.header.stamp = rospy.Time.now()            
            return req
        
        smach.StateMachine.add('MOVE_ASUS_NAV',MoveAsus.getNavigationInstance(),
                transitions={'succeeded':'SET_GOAL',
                             'aborted':'SET_GOAL'}
        )
        
        smach.StateMachine.add('SET_GOAL',
            smach_ros.ServiceState('/bender/nav/goal_server/' + service_str,
                NavGoal,
                request_cb = goal_request_cb,
                input_keys = ['goal']
            ),
            transitions = {
                'succeeded':'CHECK_REACH',
                'aborted':'SET_GOAL'
            },
            remapping = {
                'goal':'goal_pose'
            }
        )
        
        smach.StateMachine.add('CHECK_REACH', CheckReach.CheckReach(),
                transitions = {
                        'reached':'succeeded',
                        'not_reached':'SET_GOAL',
                        'aborted':'aborted'
                }
        )
        
    return sm
