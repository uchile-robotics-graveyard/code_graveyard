#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *
from geometry_msgs.msg import PoseStamped

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

# define state GrabObject
class GrabObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['position'],
                             output_keys=['selected_arm'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GrabObject')
        
        selected_arm = ""

        if userdata.position.pose.position.y > 0:
            selected_arm = "/left_arm"
        else:
            selected_arm = "/right_arm"

        selected_arm = "/right_arm"
        userdata.selected_arm = selected_arm

        try:
            pre1 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion1', Empty)
            pre1.wait_for_service()
            print "/posicion_premanipulacion1"
            resp3 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            pre2 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion2', Empty)
            pre2.wait_for_service()
            print "/posicion_premanipulacion2"
            resp4 = pre2()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            abrir = rospy.ServiceProxy(selected_arm + '/abrir_grip', Empty)
            abrir.wait_for_service()
            print "abrir_grip"
            resp4 = abrir()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            plan1 = rospy.ServiceProxy(selected_arm + '/grasp', PlanningGoalCartesian)
            plan1.wait_for_service()
            print "grasp "+ str(userdata.position.pose.position.x)+" "+str(userdata.position.pose.position.y)+" "+str(userdata.position.pose.position.z)
            resp5 = plan1(userdata.position.pose.position.x*100,userdata.position.pose.position.y*100,userdata.position.pose.position.z*100)
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
            cerrar = rospy.ServiceProxy(selected_arm + '/cerrar_grip', LoadMode)
            cerrar.wait_for_service()
            object_taken = cerrar(0)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        rospy.sleep(3)

        try:
            post1 = rospy.ServiceProxy(selected_arm + '/posicion_postmanipulacion1', Empty)
            post1.wait_for_service()
            resp8 = post1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            pre1.wait_for_service()
            resp9 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            inicial = rospy.ServiceProxy(selected_arm + '/posicion_inicial', Empty)
            inicial.wait_for_service()
            resp10 = inicial()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        print "DEBUG INFO: "
        print object_taken

        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                            input_keys = ['position'],
                            output_keys = ['selected_arm'])

    # Dummy userdata
   # sm.userdata.position = [60,1,100]
    sm.userdata.selected_arm = "/right_arm"

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
                           transitions={'succeeded':'GRAB_OBJECT'})

        smach.StateMachine.add('GRAB_OBJECT',GrabObject(),
                           remapping={'position':'position'})
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('leaveobj')

    sm = getInstance()
    ud = smach.UserData()

    p = PoseStamped()
    p.pose.position.x, p.pose.position.y, p.pose.position.z = 0.6, 0.1, 0.9
    ud.position = p

    # introspection server
    sis = smach_ros.IntrospectionServer('leaveobj', sm, '/leaveobjSM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()