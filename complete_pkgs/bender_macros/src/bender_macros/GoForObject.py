#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf

from bender_srvs.srv import String as string_srv
from bender_macros.nav import ApproachToTable
from bender_macros.nav import GoToPlace 
from bender_macros.skills import Pick_table


from bender_utils.ros import benpy
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
            output_keys=['object_class_location'])
        
        self.get_class_client = benpy.ServiceProxy('/bender/utils/mapper/get_class', string_srv)
        self.get_class_location_client = benpy.ServiceProxy('/bender/utils/mapper/get_class_location', string_srv)

        
    def execute(self, userdata):
        
        res = self.get_class_client(data=userdata.object_name)
        print res.data
        res = self.get_class_location_client(data=res.data)
        
        print res.data
        userdata.object_class_location = res.data
        

        return 'succeeded'

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():
    
    sm = smach.StateMachine(
        outcomes = ['succeeded','not_grabbed','not_found','aborted','preempted'],
        input_keys = ['object_name'],
        output_keys = ['selected_arm']
    )
    
    sm.userdata.map_name = 'map.sem_map'
    sm.userdata.selected_arm = ''
    sm.userdata.distance_approach_table = 0.55

    with sm:
        
        smach.StateMachine.add('GET_OBJECT_LOCATION',GetObjectLocation(),
            transitions = {'succeeded':'MOVE_TO_PICK_LOCATION'},
            remapping = {'object_name':'object_name',
                         'object_class_location':'object_class_location'}
        )
        smach.StateMachine.add('MOVE_TO_PICK_LOCATION',ApproachToTable.getInstance(),
               transitions={'succeeded':'PICK_OBJECT'},
               remapping={'table_name':'object_class_location',
                          'map_name':'map_name',
                          'desired_distance':'distance_approach_table'})        
        smach.StateMachine.add('PICK_OBJECT',Pick_table.getInstance(),
            transitions = {'succeeded':'succeeded',
            'notGrabb':'not_grabbed'},
            remapping = {'object_name':'object_name'}
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
    ud.object_name = 'pringles'
    
    # introspection server
    sis = smach_ros.IntrospectionServer('bring_object', sm, '/BRING_OBJECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
