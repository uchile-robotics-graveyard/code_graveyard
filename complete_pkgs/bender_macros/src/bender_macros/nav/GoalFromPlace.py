#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.knowledge import PoseMapSetup

from geometry_msgs.msg import PoseStamped
from bender_msgs.msg import SemanticObject
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_srvs.srv import SemMap
from bender_srvs.srv import SemMapRequest
from bender_srvs.srv import String
from bender_srvs.srv import StringRequest


def getMachineInstance(goal_from_pose_machine):
        
    sm = smach.StateMachine(
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['place_name','map_name']
    )
    
    with sm:
        
        @smach.cb_interface(input_keys=['place_name'])
        def place_request_cb(userdata, request):
            place_request = SemMapRequest()
            place_request.id = userdata.place_name
            return place_request

        @smach.cb_interface(output_keys=['place_pose'])
        def place_response_cb(userdata, response):

            place_pose = PoseStamped()
            place_pose.header.frame_id = response.data[0].frame_id
            place_pose.pose = response.data[0].pose
            userdata.place_pose = place_pose
            
        smach.StateMachine.add('SETUP', PoseMapSetup.Setup(),
                transitions = {'succeeded':'GET_PLACE_POSE'},
                remapping = {'map_name':'map_name'})

        smach.StateMachine.add('GET_PLACE_POSE',
                smach_ros.ServiceState('/bender/knowledge/pose_server/get',
                        SemMap,
                        request_cb = place_request_cb,
                        response_cb = place_response_cb,
                        input_keys = ['place_name'],
                        output_keys = ['place_pose']
                ),
                transitions = {'succeeded':'GOAL_TO_POSE'},
                remapping = {'place_name':'place_name',
                        'place_pose':'place_pose'
                }
        )
        
        smach.StateMachine.add('GOAL_TO_POSE',  goal_from_pose_machine,
                transitions = {'succeeded':'succeeded'},
                remapping = {'goal_pose':'place_pose'}
        )        

    return sm
