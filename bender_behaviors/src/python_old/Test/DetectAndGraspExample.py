#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *

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

        plane_height = 80
        selected_arm = userdata.selected_arm
        
        if userdata.selected_type != "unknown":

            try:
                planeon = rospy.ServiceProxy('/planeOn', planeOn)
                planeon.wait_for_service()
                resp1 = planeon(0)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return 'aborted'

            try:
                sifton = rospy.ServiceProxy('/siftOn', siftOn)
                sifton.wait_for_service()
                resp2 = sifton()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return 'aborted'

            try:
                planeheight = rospy.ServiceProxy('/arm_vision_interface/plane_height', PlaneHeight)
                planeheight.wait_for_service()
                plane_height_msg = planeheight()
                plane_height = plane_height_msg.height
                print "Detected plane height to place object: ", plane_height
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return 'aborted'

            time.sleep(1)

            try:
                planeoff = rospy.ServiceProxy('/planeOff', planeOff)
                planeoff.wait_for_service()
                resp1 = planeoff(0)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return 'aborted'

            try:
                siftoff = rospy.ServiceProxy('/siftOff', siftOff)
                siftoff.wait_for_service()
                resp1 = siftoff(0)
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
            resp5 = plan1(70,0,plane_height+7)
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

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','canceled'])
    
    sm.userdata.object_position = {}
    sm.userdata.object_id = []
    sm.userdata.object_type = []
    sm.userdata.object_status = []

    sm.userdata.selected_position = []
    sm.userdata.selected_type = ""
    sm.userdata.selected_arm = "/right_arm"

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'POSITION_AND_DETECT'})

        smach.StateMachine.add('POSITION_AND_DETECT',PositionAndDetect.getInstance(),
                           transitions={'succeeded':'CHOOSE_OBJECT'},
                           remapping={'object_position':'object_position',
                                      'object_id':'object_id',
                                      'object_type':'object_type',
                                      'object_status':'object_status'})

        smach.StateMachine.add('CHOOSE_OBJECT',ChooseObject(),
                           transitions={'succeeded':'PICK_UP'},
                           remapping={'object_position':'object_position',
                                      'object_type':'object_type',
                                      'object_status':'object_status',
                                      'selected_position':'selected_position',
                                      'selected_type':'selected_type'})

        smach.StateMachine.add('PICK_UP',GraspObject.getInstance(),
                           transitions={'notGrabbed':'PICK_UP',
                                        'succeeded':'PLACE_OBJECT'},
                           remapping={'position':'selected_position',
                                      'selected_arm':'selected_arm'})

        smach.StateMachine.add('PLACE_OBJECT',PlaceObject(),
                           remapping={'selected_type':'selected_type',
                                      'selected_arm':'selected_arm'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('pick_and_place')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('pick_and_place', sm, '/PICK_AND_PLACE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
