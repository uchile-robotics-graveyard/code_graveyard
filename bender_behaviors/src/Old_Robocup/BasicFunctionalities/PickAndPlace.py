#!/usr/bin/env python

import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_macros.nav import GoToPlace
from bender_macros.nav import ApproachToTable
from bender_macros.vision import PositionAndDetect
from bender_macros.arm import GraspObject

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        rospy.sleep(0.5)
        return 'succeeded'


class GetObjectLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['object_class'],
            output_keys=['object_class_location'])
        
        self.get_class_location_client = rospy.ServiceProxy('/bender/utilities/mapper/get_class_location', string_srv)

        
    def execute(self, userdata):

        self.get_class_location_client.wait_for_service()
        res = self.get_class_location_client(data=userdata.object_class)

        userdata.object_class_location = res.data
        
        return 'succeeded'



# define state ChooseObject
class ChooseObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','canceled'],
                             input_keys=['object_position','object_type','object_status'],
                             output_keys=['selected_position','selected_type'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')

        if not "inRange" in userdata.object_status:
            return 'canceled'
        
        for i in range(len(userdata.object_type)):
            if userdata.object_type[i] != "unknown" and userdata.object_status[i] == "inRange":
                userdata.selected_type = userdata.object_type[i]
                userdata.selected_position = [userdata.object_position['x'][i], userdata.object_position['y'][i], userdata.object_position['z'][i]]
                print "known object found"
                return 'succeeded'

        idx = userdata.object_status.index("inRange")
        userdata.selected_type = userdata.object_type[idx]
        userdata.selected_position = [userdata.object_position['x'][idx], userdata.object_position['y'][idx], userdata.object_position['z'][idx]]        
        return 'succeeded'

# define state PlaceObject
class PlaceObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['selected_type','selected_arm'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')

        
        try:
            pH = rospy.ServiceProxy('/heighttable', heighttable)
            pH.wait_for_service()
            respH = pH()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'
	  
        plane_height = respH.height
        #if userdata.selected_type == "drinks":
         #   plane_height = 82

       # if userdata.selected_type == "food":
        #    plane_height == 100
        # - - - - - - - - - - - - - - - - - - - - - - - 

        selected_arm = userdata.selected_arm
                    
        try:
            pre2 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion1', Dummy)
            pre2.wait_for_service()
            resp4 = pre2()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            pre2 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion2', Dummy)
            pre2.wait_for_service()
            resp4 = pre2()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            plan1 = rospy.ServiceProxy(selected_arm + '/grasp', PlanningGoalCartesian)
            plan1.wait_for_service()
            resp5 = plan1(70,0,plane_height+2)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            orient_grip = rospy.ServiceProxy(selected_arm + '/orientar_grip', Dummy)
            orient_grip.wait_for_service()
            resp6 = orient_grip()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        rospy.sleep(1)

        try:
            abrir = rospy.ServiceProxy(selected_arm + '/abrir_grip', Dummy)
            abrir.wait_for_service()
            resp7 = abrir()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            post1 = rospy.ServiceProxy(selected_arm + '/posicion_postmanipulacion1', Dummy)
            post1.wait_for_service()
            resp8 = post1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            cerrar = rospy.ServiceProxy(selected_arm + '/mover_grip_ang', AngVel)
            cerrar.wait_for_service()
            req = AngVelRequest()
            req.angle = 0.3
            req.velocity = 0.4
            resp9 = cerrar(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            inicial = rospy.ServiceProxy(selected_arm + '/posicion_inicial', Dummy)
            inicial.wait_for_service()
            resp10 = inicial()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'

# define state Apologize
class Apologize(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Apologize')
        try:
            apologize = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize', synthesize)
            apologize.wait_for_service()
            resp1 = apologize('I am sorry, I could not find any objects. I will wait here until you allow me to proceed with the next test.')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        while rospy.get_param('/bender/speech/synthesizer/talking'):
            rospy.sleep(0.5)
        return 'succeeded'


# define state Apologize2
class Apologize2(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Apologize2')
        try:
            apologize = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize', synthesize)
            apologize.wait_for_service()
            resp1 = apologize('I am sorry, I can not reach the detected objects. I will wait here until you allow me to proceed with the next test.')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        while rospy.get_param('/bender/speech/synthesizer/talking'):
            rospy.sleep(0.5)
        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys=['map_name','pick_place']
    )
    
    sm.userdata.object_position = {}
    sm.userdata.object_id = []
    sm.userdata.object_type = []
    sm.userdata.object_status = []
    sm.userdata.object_class_location = ""
    sm.userdata.selected_position = []
    sm.userdata.selected_type = ""
    sm.userdata.selected_arm = "/right_arm"

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
               transitions={'succeeded':'MOVE_TO_PICK_LOCATION'})

        smach.StateMachine.add('MOVE_TO_PICK_LOCATION',GoToPlace.getInstance(),
               transitions={'succeeded':'WAIT_POSITION'},
               remapping={'place_name':'pick_place',
                          'map_name':'map_name'})

        smach.StateMachine.add('WAIT_POSITION',ApproachToTable.getInstance(),
               transitions={'succeeded':'POSITION_AND_DETECT'})

        smach.StateMachine.add('POSITION_AND_DETECT',PositionAndDetect.getInstance(),
               transitions={'succeeded':'CHOOSE_OBJECT',
                            'canceled':'APOLOGIZE'},
               remapping={'object_position':'object_position',
                          'object_id':'object_id',
                          'object_type':'object_type',
                          'object_status':'object_status'})

        smach.StateMachine.add('CHOOSE_OBJECT',ChooseObject(),
               transitions={'succeeded':'PICK_UP',
                            'canceled':'APOLOGIZE2'},
               remapping={'object_position':'object_position',
                          'object_type':'object_type',
                          'object_status':'object_status',
                          'selected_position':'selected_position',
                          'selected_type':'selected_type'})

        smach.StateMachine.add('PICK_UP',GraspObject.getInstance(),
               transitions={'notGrabbed':'POSITION_AND_DETECT',
                            'succeeded':'GET_OBJECT_LOCATION'},
               remapping={'position':'selected_position'})

        smach.StateMachine.add('GET_OBJECT_LOCATION',GetObjectLocation(),
                transitions = {'succeeded':'MOVE_TO_PLACE_LOCATION'},
                remapping = {'object_class':'selected_type',
                         'object_class_location':'object_class_location'}
        )                

        smach.StateMachine.add('MOVE_TO_PLACE_LOCATION',GoToPlace.getInstance(),
               transitions={'succeeded':'PLACE_OBJECT',
                            'aborted':'MOVE_TO_PLACE_LOCATION'},
               remapping={'place_name':'object_class_location',
                          'map_name':'map_name'})

        smach.StateMachine.add('PLACE_OBJECT',PlaceObject(),
               remapping={'selected_type':'selected_type',
                          'selected_arm':'selected_arm'})
        smach.StateMachine.add('APOLOGIZE',Apologize(),
               transitions={'succeeded':'succeeded'})

        smach.StateMachine.add('APOLOGIZE2',Apologize2(),
               transitions={'succeeded':'succeeded'})
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('pick_and_place')

    sm = getInstance()
    ud = smach.UserData()
    ud.map_name = "map.sem_map"
    ud.pick_place = "kitchen_table"

    # introspection server
    sis = smach_ros.IntrospectionServer('pick_and_place', sm, '/PICK_AND_PLACE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
