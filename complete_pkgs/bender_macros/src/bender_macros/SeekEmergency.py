#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from bender_macros.nav import GoToPlace
from bender_macros import LookForEmergency

# solo para testeo
import cv2
import cv_bridge
import numpy as np

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class PrepareNextLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['place_name'] 
        )

        self.place_idx = 1
        self.max_place_idx = 4

    def execute(self, userdata):
        
        if self.place_idx <= self.max_place_idx:
            
            userdata.place_name = 'es_detect_place_' + str(self.place_idx)
            self.place_idx += 1
            return 'succeeded'
        else:
            return 'aborted'

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    sm = smach.StateMachine(
        outcomes = ['succeeded','aborted','preempted','timeout'],
        output_keys = ['person_pose','images', 'image_captions'],
        input_keys = ['images', 'image_captions','map_name']
    )
    
    sm.userdata.person_pose = PoseStamped()
    sm.userdata.machine_state = 'working'
    sm.userdata.images = []
    sm.userdata.image_captions = []

    # sub machines - states    
    go_to_place_sm = GoToPlace.getInstance()

    with sm:

        smach.StateMachine.add('GET_NEXT_LOCATION',PrepareNextLocation(),
            transitions={
                'succeeded':'GO_TO_NEXT_LOCATION',
                'aborted':'aborted'
            },
            remapping = { 'place_name':'place_name' }
        )
        
        smach.StateMachine.add('GO_TO_NEXT_LOCATION',go_to_place_sm,
            transitions={'succeeded':'LOOK_FOR_EMERGENCY'},
            remapping = { 'place_name':'place_name',
                          'map_name':'map_name'}
        )
        
        smach.StateMachine.add('LOOK_FOR_EMERGENCY',LookForEmergency.getInstance(),
           transitions = {'succeeded':'succeeded',
                          'aborted':'GET_NEXT_LOCATION',
                          'timeout':'timeout'},
           remapping = {'person_pose':'person_pose',
                        'images':'images',
                        'image_captions':'image_captions'}
        )
        
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('seek_emergency')

    sm = getInstance()
    
    # dummy userdata
    ud = smach.UserData()
    ud.map_name = 'amtc.sem_map'
    ud.images = []
    ud.image_captions = []

    # introspection server
    sis = smach_ros.IntrospectionServer('seek_emergency', sm, '/SEEK_EMERGENCY')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
