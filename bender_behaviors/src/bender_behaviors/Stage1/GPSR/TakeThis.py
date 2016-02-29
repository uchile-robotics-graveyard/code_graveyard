#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.nav import GoToPlace


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


class Adjust(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['selected_arm'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Adjust')

        selected_arm = userdata.selected_arm

        try:
            pre1 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion1', Empty)
            pre1.wait_for_service()
            resp3 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            pre2 = rospy.ServiceProxy(selected_arm + '/posicion_premanipulacion2', Empty)
            pre2.wait_for_service()
            resp4 = pre2()
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
	return 'succeeded'
  
  
class Take(smach.State):
  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','notGrabbed','lasttime','aborted','preempted'],
                             input_keys=['selected_arm'])
        self.count = 0
        self.maxcount = 3

    def execute(self, userdata):
        rospy.loginfo('Executing state Take')

        selected_arm = userdata.selected_arm

        self.count +=1 
        if self.count == self.maxcount:
            return 'lasttime'	  


        try:
            abrir = rospy.ServiceProxy(selected_arm + '/abrir_grip', Empty)
            abrir.wait_for_service()
            resp4 = abrir()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        try:
            sp1 = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize', synthesize)
            sp1.wait_for_service()  
            
            intro = 'por favor, colocar el objeto en el gripper'
            resp1 = sp1(intro)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        rospy.sleep(5)
            

        try:
            cerrar = rospy.ServiceProxy(selected_arm + '/cerrar_grip', LoadMode)
            cerrar.wait_for_service()
            object_taken = cerrar(0)
            
            if self.count != self.maxcount+1 and not object_taken.taken:
                return 'notGrabbed'
	    
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        rospy.sleep(3)

        try:
            post1 = rospy.ServiceProxy(selected_arm + '/posicion_inicial', Empty)
            post1.wait_for_service()
            resp8 = post1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        if self.count == self.maxcount+1 and not object_taken.taken:
            return 'preempted'
        return 'succeeded'


class LastTime(smach.State):
  
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['selected_arm'])
        self.count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state LastTime')

        try:
            sp1 = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize', synthesize)
            sp1.wait_for_service()  
            
            intro = 'Esta es tu ultima oportunidad.'
            resp1 = sp1(intro)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        rospy.sleep(3)

        return 'succeeded'


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted','notGrabbed'],
                            output_keys = ['selected_arm'])

    # Dummy userdata
    sm.userdata.selected_arm = "/right_arm"

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
                           transitions={'succeeded':'ADJUST'})
        smach.StateMachine.add('ADJUST',Adjust(),
			   transitions={'succeeded':'TAKE'})
        smach.StateMachine.add('TAKE',Take(),
			   transitions={'notGrabbed':'TAKE',
                            'lasttime' : 'LastTime'})
        smach.StateMachine.add('LastTime',LastTime(),
               transitions={'succeeded':'TAKE'})
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('TakeThis')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('TakeThis', sm, '/TakeThisSM')
    sis.start()
    outcome = sm.execute()
    sis.stop()