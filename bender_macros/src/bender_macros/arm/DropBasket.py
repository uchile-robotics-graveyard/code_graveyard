#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *

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

# define state DropBasket
class DropBasket(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['basket_side','selected_arm'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DropBasket')

        selected_arm = userdata.selected_arm
        if selected_arm == '/left_arm':    
            if userdata.basket_side == 'left':
                conf = [0.431048601396,-0.349747619638,-0.62381885374,1.79475752183]
            else:
                conf = [0.105844674364,-0.242368964486,-0.89993539556,1.65976721249]
        else:
            if userdata.basket_side == 'left':
                conf = [-0.105844674364,0.242368964486,-0.89993539556,1.65976721249]
            else:
                conf = [-0.431048601396,0.349747619638,-0.62381885374,1.79475752183]

        try:
            pre1 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion1', Empty)
            pre1.wait_for_service()
            resp3 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            plan1 = rospy.ServiceProxy(selected_arm + '/plan_state', PlanningGoalState)
            plan1.wait_for_service()
            resp5 = plan1(conf[0],conf[1],conf[2],conf[3])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            orient_grip = rospy.ServiceProxy(selected_arm + '/orientar_grip', Empty)
            orient_grip.wait_for_service()
            resp6 = orient_grip()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            abrir = rospy.ServiceProxy(selected_arm + '/abrir_grip', Empty)
            abrir.wait_for_service()
            resp4 = abrir()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        rospy.sleep(2);

        try:
            cerrar = rospy.ServiceProxy(selected_arm + '/mover_grip_ang', AngVel)
            cerrar.wait_for_service()
            object_taken = cerrar(0.3,0.3)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            resp5 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            inicial = rospy.ServiceProxy(selected_arm + '/posicion_inicial', Empty)
            inicial.wait_for_service()
            resp6 = inicial()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                             input_keys=['basket_side','selected_arm'])

    sm.userdata.basket_side = "right"
    sm.userdata.selected_arm = "/right_arm"

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'DROP_BASKET'})

        smach.StateMachine.add('DROP_BASKET',DropBasket(),
                           transitions={'succeeded':'succeeded'})
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('drop_in_basket')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('drop_in_basket', sm, '/DROP_IN_BASKET_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()