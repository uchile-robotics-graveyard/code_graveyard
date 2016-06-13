#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf

from bender_srvs.srv import String as string_srv
from bender_macros.arm import GraspObject
from bender_macros import LookForObject
from bender_macros.nav import ApproachToTable
from bender_macros.nav import GoToPlace 

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class foo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.sleep(0.5)
        
        return 'succeeded'
    

class WaitPosition(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WaitPosition')
        rospy.sleep(5)
        return 'succeeded'
   

class GetObjectLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['object_name'],
            output_keys=['object_class_location','confirmation_ask'])
        
        self.get_class_client = rospy.ServiceProxy('/bender/utils/mapper/get_class', string_srv)
        self.get_class_location_client = rospy.ServiceProxy('/bender/utils/mapper/get_class_location', string_srv)

        
    def execute(self, userdata):
        
        self.get_class_client.wait_for_service()
        res = self.get_class_client(data=userdata.object_name)

        self.get_class_location_client.wait_for_service()
        res2 = self.get_class_location_client(data=res.data)
        
	userdata.object_class_location = res2.data
        userdata.confirmation_ask  = ''
        
        return 'succeeded'

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    
    sm = smach.StateMachine(
        outcomes = ['succeeded','not_grabbed','not_found','aborted','preempted'],
        input_keys = ['object_name'],
        output_keys = ['grasp_arm']
    )
    
    sm.userdata.objects_map = 'map.sem_map'
    sm.userdata.grasp_arm = ''
    sm.userdata.confirmation_ask = ''
    
    with sm:
        
        smach.StateMachine.add('GET_OBJECT_LOCATION',GetObjectLocation(),
            transitions = {'succeeded':'ASK_FOR_CONFIRMATION'},
            remapping = {'object_name':'object_name',
                         'object_class_location':'object_class_location'}
        )
	    
        smach.StateMachine.add('ASK_FOR_CONFIRMATION',AskForConfirmation.getInstance(),
            transitions = {'succeeded':'MOVE_TO_PICK_LOCATION',
                            'aborted':'Ask_Object_Location'}
        )
        smach.StateMachine.add('Ask_Object_Location',AskObjectLocation(),
           transitions={'succeeded':'ASK_FOR_CONFIRMATION'},
           remapping={'place_name':'object_class_location'}
        )
        smach.StateMachine.add('MOVE_TO_PICK_LOCATION',GoToPlace.getInstance(),
           transitions={'succeeded':'APPROACH_TO_TABLE'},
           remapping={'place_name':'object_class_location',
                      'map_name':'objects_map'}
        )

        smach.StateMachine.add('APPROACH_TO_TABLE',ApproachToTable.getInstance(),
            transitions = {'succeeded':'LOOK_FOR_OBJECT'},
            remapping = {'table_name':'object_class_location',
                         'map_name':'objects_map'}
        )
      
        
        smach.StateMachine.add('LOOK_FOR_OBJECT',LookForObject.getInstance(),
            transitions = {'succeeded':'GRAB_OBJECT',
                           'not_found':'not_found'},
            remapping = {'required_object':'object_name',
                         'object_position':'object_position'}
        )
        
        smach.StateMachine.add('GRAB_OBJECT',GraspObject.getInstance(),
            transitions = {'succeeded':'succeeded',
                           'notGrabbed':'not_grabbed'},
            remapping = {'position':'object_position',
                         'selected_arm':'grasp_arm'}
        )

        #ud = smach.UserData()
        #ud.object_name = 'coca'
        #ud.person_pose = person_pose
        
        #sm.set_initial_state(['LOOK_FOR_OBJECT'], ud)

        
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('bring_object')

    sm = getInstance()

    ud = smach.UserData()
    ud.object_name = 'cola'
    
    # introspection server
    sis = smach_ros.IntrospectionServer('bring_object', sm, '/BRING_OBJECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
