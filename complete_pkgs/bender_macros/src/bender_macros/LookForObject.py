#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf

from math import sqrt
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.vision import DetectObjects


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class foo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.sleep(0.5)
        
        return 'succeeded'


class LookNeighborhood(smach.State):
    '''
    TODO: Por ahora es dummy... se ejecuta 3 veces
    '''

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['detect_angles']
        )
       
        rotate_client = rospy.ServiceProxy('/bender/nav/goal_server/rotate', NavGoal)

        self.counter = 0
        
    def execute(self, userdata):
        
        if self.counter >= len(userdata.detect_angles):
            return 'aborted'
        
        rospy.loginfo("trying to detect again . . . attemp number: " + str(self.counter))

        req = NavGoalRequest()
        req.rotation = userdata.detect_angles[self.counter]

        self.rotate_client.wait_for_service()

        resp = rotate_client(req)
        rospy.sleep(3)


        self.counter += 1

        return 'succeeded'
        
class ChooseObject(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','no_match','aborted','preempted'],
            input_keys = ['required_object','detection_ids','detection_positions','detection_types','detection_status'],
            output_keys = ['selected_position']
        )
        
    def execute(self, userdata):

        rospy.loginfo('Executing state ChooseObject')

        # look for object in detections array
        if not userdata.required_object in userdata.detection_ids:
            return 'no_match'

        for i in range(len(userdata.detection_types)):
            print  userdata.detection_status[i] 
            print userdata.required_object
            print userdata.detection_ids[i] 
            print ""
            if userdata.detection_ids[i] == userdata.required_object and userdata.detection_status[i] == "in_range":

                userdata.selected_position = [
                    userdata.detection_positions['x'][i],
                    userdata.detection_positions['y'][i],
                    userdata.detection_positions['z'][i]
                ]

                return 'succeeded'

        # en realidad, aqui hay match, pero el obj no esta en rango.
        # se propone mejorar la maquina, para decirle que se mueva
        # hacia la direccion donde se encontro el objeto
        return 'no_match'

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    
    sm = smach.StateMachine(
        outcomes = ['succeeded','not_found','aborted','preempted'],
        input_keys = ['required_object'],
        output_keys = ['object_position']
    )

    sm.userdata.detect_angles =  [0,-15,30,-15]
    sm.userdata.object_position = []

    with sm:

        smach.StateMachine.add('DETECT_OBJECTS', DetectObjects.getInstance(),
            transitions = {'succeeded':'CHOOSE_OBJECT',
                           'no_detections':'LOOK_NEIGHBORHOOD'},
            remapping = {'object_positions':'detection_positions',
                         'object_ids':'detection_ids',
                         'object_types':'detection_types',
                         'object_status':'detection_status'}
        )
        
        smach.StateMachine.add('CHOOSE_OBJECT',ChooseObject(),
            transitions = {'succeeded':'succeeded',
                           'no_match':'LOOK_NEIGHBORHOOD'},
            remapping = {'required_object':'required_object',
                         'detection_ids':'detection_ids',
                         'detection_positions':'detection_positions',
                         'detection_types':'detection_types',
                         'detection_status':'detection_status',
                         'selected_position':'object_position'
            }

        )
        
        smach.StateMachine.add('LOOK_NEIGHBORHOOD',LookNeighborhood(),
            transitions = {'succeeded':'DETECT_OBJECTS',
                           'aborted':'not_found'},
            remapping = {'detect_angles':'detect_angles'}
        )
        
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('bring_object')

    sm = getInstance()

    ud = smach.UserData()
    ud.required_object = 'coca'

    # introspection server
    sis = smach_ros.IntrospectionServer('bring_object', sm, '/BRING_OBJECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
