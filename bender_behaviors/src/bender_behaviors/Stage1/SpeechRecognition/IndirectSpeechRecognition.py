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
from bender_macros.nav  import RotateRobot
from bender_srvs.srv import load_dictionary_service

class Setup(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
 
    def execute(self, userdata):
        try:
            response = rospy.ServiceProxy('bender/speech/recognizer/load_dictionary', load_dictionary_service)
            response('quest')
            esponse = rospy.ServiceProxy('bender/speech/recognizer2/load_dictionary', load_dictionary_service)
            esponse('quest')
            return 'succeeded'
        except rospy.ServiceException:
            return 'aborted'

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        input_keys=['timeout']
    )

    sm.userdata.talk_timeover_turn = 'I could not get the question, i will turn around'
    sm.userdata.timeout_timeover_turn = 5
    sm.userdata.talk_timeover = 'I could not get the question'
    sm.userdata.timeout_timeover = 3
    sm.userdata.turn_around_angle = 180
    sm.userdata.talk_move_front = 'Please put behind me and wait'
    sm.userdata.timeout_move_front = 5
    
    with sm:
        smach.StateMachine.add('SETUP', Setup(),
            transitions={'succeeded':'Q1'},
        )
        
        smach.StateMachine.add('Q1', AskQuestion.ask(text='6',speech='indirect'),transitions={'understand':'Q2','aborted':'APOLOGIZE1','time over':'APOLOGIZE1'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q2', AskQuestion.ask(text='7',speech='indirect'),transitions={'understand':'Q3','aborted':'APOLOGIZE2','time over':'APOLOGIZE2'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q3', AskQuestion.ask(text='8',speech='indirect'),transitions={'understand':'Q4','aborted':'APOLOGIZE3','time over':'APOLOGIZE3'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q4', AskQuestion.ask(text='9',speech='indirect'),transitions={'understand':'Q5','aborted':'APOLOGIZE4','time over':'APOLOGIZE4'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q5', AskQuestion.ask(text='10',speech='indirect'),transitions={'understand':'succeeded','aborted':'APOLOGIZE5','time over':'APOLOGIZE5'},remapping={'timeout':'timeout'})

        smach.StateMachine.add('APOLOGIZE1',TalkState.getInstance(),transitions={'succeeded':'TURN_AROUND1'},remapping={'text':'talk_timeover_turn','timeout':'timeout_timeover_turn'})
        smach.StateMachine.add('APOLOGIZE2',TalkState.getInstance(),transitions={'succeeded':'TURN_AROUND2'},remapping={'text':'talk_timeover_turn','timeout':'timeout_timeover_turn'})
        smach.StateMachine.add('APOLOGIZE3',TalkState.getInstance(),transitions={'succeeded':'TURN_AROUND3'},remapping={'text':'talk_timeover_turn','timeout':'timeout_timeover_turn'})
        smach.StateMachine.add('APOLOGIZE4',TalkState.getInstance(),transitions={'succeeded':'TURN_AROUND4'},remapping={'text':'talk_timeover_turn','timeout':'timeout_timeover_turn'})
        smach.StateMachine.add('APOLOGIZE5',TalkState.getInstance(),transitions={'succeeded':'TURN_AROUND5'},remapping={'text':'talk_timeover_turn','timeout':'timeout_timeover_turn'})

        smach.StateMachine.add('TURN_AROUND1',RotateRobot.getInstance(),transitions={'succeeded':'Q1_d','aborted':'Q1_d'},remapping={'angle':'turn_around_angle'})
        smach.StateMachine.add('TURN_AROUND2',RotateRobot.getInstance(),transitions={'succeeded':'Q2_d','aborted':'Q2_d'},remapping={'angle':'turn_around_angle'})
        smach.StateMachine.add('TURN_AROUND3',RotateRobot.getInstance(),transitions={'succeeded':'Q3_d','aborted':'Q3_d'},remapping={'angle':'turn_around_angle'})
        smach.StateMachine.add('TURN_AROUND4',RotateRobot.getInstance(),transitions={'succeeded':'Q4_d','aborted':'Q4_d'},remapping={'angle':'turn_around_angle'})
        smach.StateMachine.add('TURN_AROUND5',RotateRobot.getInstance(),transitions={'succeeded':'Q5_d','aborted':'Q5_d'},remapping={'angle':'turn_around_angle'})
        
        smach.StateMachine.add('Q1_d', AskQuestion.ask(text='6',speech='indirect'),transitions={'understand':'Q2','aborted':'APOLOGIZE1_d','time over':'APOLOGIZE1_d'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q2_d', AskQuestion.ask(text='7',speech='indirect'),transitions={'understand':'Q3','aborted':'APOLOGIZE2_d','time over':'APOLOGIZE2_d'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q3_d', AskQuestion.ask(text='8',speech='indirect'),transitions={'understand':'Q4','aborted':'APOLOGIZE3_d','time over':'APOLOGIZE3_d'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q4_d', AskQuestion.ask(text='9',speech='indirect'),transitions={'understand':'Q5','aborted':'APOLOGIZE4_d','time over':'APOLOGIZE4_d'},remapping={'timeout':'timeout'})
        smach.StateMachine.add('Q5_d', AskQuestion.ask(text='10',speech='indirect'),transitions={'understand':'succeeded','aborted':'APOLOGIZE5_d','time over':'APOLOGIZE5_d'},remapping={'timeout':'timeout'})

        smach.StateMachine.add('APOLOGIZE1_d',TalkState.getInstance(),transitions={'succeeded':'TALK_MOVE_FRONT1'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE2_d',TalkState.getInstance(),transitions={'succeeded':'TALK_MOVE_FRONT2'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE3_d',TalkState.getInstance(),transitions={'succeeded':'TALK_MOVE_FRONT3'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE4_d',TalkState.getInstance(),transitions={'succeeded':'TALK_MOVE_FRONT4'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
        smach.StateMachine.add('APOLOGIZE5_d',TalkState.getInstance(),transitions={'succeeded':'succeeded'},remapping={'text':'talk_timeover','timeout':'timeout_timeover'})
    
        smach.StateMachine.add('TALK_MOVE_FRONT1',TalkState.getInstance(),transitions={'succeeded':'Q2'},remapping={'text':'talk_move_front','timeout':'timeout_move_front'})
        smach.StateMachine.add('TALK_MOVE_FRONT2',TalkState.getInstance(),transitions={'succeeded':'Q3'},remapping={'text':'talk_move_front','timeout':'timeout_move_front'})
        smach.StateMachine.add('TALK_MOVE_FRONT3',TalkState.getInstance(),transitions={'succeeded':'Q4'},remapping={'text':'talk_move_front','timeout':'timeout_move_front'})
        smach.StateMachine.add('TALK_MOVE_FRONT4',TalkState.getInstance(),transitions={'succeeded':'Q5'},remapping={'text':'talk_move_front','timeout':'timeout_move_front'})
    

    return sm


if __name__ == '__main__':

    rospy.init_node('indirect_speech')

    sm = getInstance()
    ud = smach.UserData()
    ud.timeout = 10
    sis = smach_ros.IntrospectionServer('indirect_speech', sm, '/INDIRECT_SPEECH_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
