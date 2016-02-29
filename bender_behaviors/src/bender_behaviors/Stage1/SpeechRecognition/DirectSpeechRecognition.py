#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from bender_macros.speech import Talk
from bender_macros.speech import TalkState
from bender_macros.speech import AskQuestion
from bender_srvs.srv import load_dictionary_service
from docutils.transforms.misc import Transitions

class Setup(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        try:
            response = rospy.ServiceProxy('bender/speech/recognizer/load_dictionary', load_dictionary_service)
            response('quest')
            return 'succeeded'
        except rospy.ServiceException:
            return 'aborted'

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys=['timeout']
    )

    sm.userdata.talk_timeover = 'I could not get the question'
    sm.userdata.timeout_timeover = 3

    with sm:
        smach.StateMachine.add('SETUP1', Setup(),transitions={'succeeded':'Q1'})
        smach.StateMachine.add('SETUP2', Setup(),transitions={'succeeded':'Q2'})
        smach.StateMachine.add('SETUP3', Setup(),transitions={'succeeded':'Q3'})
        smach.StateMachine.add('SETUP4', Setup(),transitions={'succeeded':'Q4'})
        smach.StateMachine.add('SETUP5', Setup(),transitions={'succeeded':'Q5'})

        smach.StateMachine.add('Q1', AskQuestion.ask(text='1',speech='direct'),transitions={'understand':'SETUP2','aborted':'APOLOGIZE1','time over':'APOLOGIZE1'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q2', AskQuestion.ask(text='2',speech='direct'),transitions={'understand':'SETUP3','aborted':'APOLOGIZE2','time over':'APOLOGIZE2'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q3', AskQuestion.ask(text='3',speech='direct'),transitions={'understand':'SETUP4','aborted':'APOLOGIZE3','time over':'APOLOGIZE3'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q4', AskQuestion.ask(text='4',speech='direct'),transitions={'understand':'SETUP5','aborted':'APOLOGIZE4','time over':'APOLOGIZE4'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q5', AskQuestion.ask(text='5',speech='direct'),transitions={'understand':'succeeded','aborted':'APOLOGIZE5','time over':'APOLOGIZE5'},remapping={'timeout':'timeout'})

        smach.StateMachine.add('APOLOGIZE1',Talk.State(),transitions={'succeeded':'Q2'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE2',Talk.State(),transitions={'succeeded':'Q3'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE3',Talk.State(),transitions={'succeeded':'Q4'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE4',Talk.State(),transitions={'succeeded':'Q5'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE5',Talk.State(),transitions={'succeeded':'succeeded'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})

    return sm

if __name__ == '__main__':

    rospy.init_node('direct_speech')

    sm = getInstance()
    ud = smach.UserData()
    ud.timeout = 10
    sis = smach_ros.IntrospectionServer('direct_speech', sm, '/DIRECT_SPEECH_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
