#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_msgs.msg import SemanticObject
from bender_srvs.srv import SemMap
from bender_srvs.srv import SemMapRequest
from bender_srvs.srv import String
from bender_srvs.srv import StringRequest


class Setup(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, 
                outcomes=['succeeded','aborted','preempted'],
                input_keys = ['map_name']
        )
        
    
    def execute(self, userdata):
        
        which_map_client = rospy.ServiceProxy('/bender/knowledge/pose_server/which', String)
        load_map_client = rospy.ServiceProxy('/bender/knowledge/pose_server/load', String)
        
        try:
            
            which_map_client.wait_for_service()
            which_resp = which_map_client()
            
            if which_resp.data != userdata.map_name:
                                
                rospy.loginfo('Current map: "' + which_resp.data +
                             '", Loading map: "' + userdata.map_name + '"'
                )
                
                load_request = StringRequest()
                load_request.data = userdata.map_name
                load_map_client.wait_for_service()
                load_map_client(load_request)    
        
            else:
                
                rospy.loginfo('Current map: "' + which_resp.data +
                             '", Loading of map is not necessary')
                
        except Exception, e:
            return 'aborted'
        
        
        return 'succeeded'
