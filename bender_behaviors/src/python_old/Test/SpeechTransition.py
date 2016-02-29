#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *

Next = False

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        dictionary = "Stage1/pickandplace"

        # Services
        startSrv = rospy.ServiceProxy('/bender/speech/recognizer/start', Empty)
        loadDictSrv = rospy.ServiceProxy('/bender/speech/recognizer/load_dictionary', load_dictionary_service)
        

        # Wait services
        print 'Waiting service: /bender/speech/recognizer/start'
        rospy.wait_for_service('/bender/speech/recognizer/start')

        print 'Waiting service: /bender/speech/recognizer/load_dictionary'
        rospy.wait_for_service('/bender/speech/recognizer/load_dictionary')

        print 'Loading dictionary'
        response = loadDictSrv(dictionary)
        started = startSrv()

        print 'running ... '

        return 'succeeded'

# define state State1
class State1(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State1')

        global Next

        # Subscriptions
        rospy.Subscriber('/bender/speech/recognizer/output', std_msgs.msg.String, recognitionCB)
        
        while not Next and not rospy.is_shutdown():
            rospy.sleep(0.1);

        Next = False

        return 'succeeded'

# define state State2
class State2(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State2')

        global Next

        # Subscriptions
        rospy.Subscriber('/bender/speech/recognizer/output', std_msgs.msg.String, recognitionCB)
        
        while not Next and not rospy.is_shutdown():
            rospy.sleep(0.1)

        Next = False

        return 'succeeded'

# define state State3
class State3(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state State3')

        global Next

        # Subscriptions
        rospy.Subscriber('/bender/speech/recognizer/output', std_msgs.msg.String, recognitionCB)
        
        while not Next and not rospy.is_shutdown():
            rospy.sleep(0.1);

        Next = False

        return 'succeeded'

def recognitionCB(recognition):

    global Next

    if recognition.data == 'bender go on':
        Next = True

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'STATE1'})

        smach.StateMachine.add('STATE1',State1(),
                           transitions={'succeeded':'STATE2'})

        smach.StateMachine.add('STATE2',State2(),
                           transitions={'succeeded':'STATE3'})

        smach.StateMachine.add('STATE3',State3(),
                           transitions={'succeeded':'STATE1'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('SpeechTransitionExample')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('speech_transition_example', sm, '/SPEECH_TRANSITION_EXAMPLE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()