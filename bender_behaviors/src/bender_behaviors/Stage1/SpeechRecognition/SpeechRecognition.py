#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from bender_macros.vision import FindFace_RGBD
from bender_macros.speech import TalkState
from bender_macros.speech import Talk
import DirectSpeechRecognition
# import IndirectSpeechRecognition

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        Talk.getInstance('I am ready to start the Speech Recognition and audio detection test', 5)
        return 'succeeded'

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  M a c h i n e    C r e a t i o n                     #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'])

    sm.userdata.recog_talk = 'I see a person in front of me'
    sm.userdata.front_talk = 'I will listen to you'
    sm.userdata.rear_talk = 'I will listen to you from my back'
    sm.userdata.timeout_talk = 3

    sm.userdata.end_talk = 'I have finished the Speech Recognition and audio detection test, bye bye'
    sm.userdata.end_timeout_talk = 5

    sm.userdata.move_talk = 'I have finished the "direct speech recognition" stage, now put behind me and wait'
    sm.userdata.move_timeout_talk = 10

    sm.userdata.wait_text = "Can you get closer, please"
    sm.userdata.wait_timeout = -1

    sm.userdata.timeout_direct_speech = 10
    sm.userdata.timeout_indirect_speech = 10

    with sm:

        smach.StateMachine.add('SETUP', Setup(),
            transitions = {'succeeded':'FIND_FACE'}
        )

        smach.StateMachine.add('FIND_FACE', FindFace_RGBD.getInstance(),
            transitions = {'Face Detected':'TALK_RECOG','Face Not Detected':'FIND_FACE'}
        )

        smach.StateMachine.add('TALK_RECOG', TalkState.getInstance(),
            transitions = {'succeeded':'TALK_FRONT','aborted':'TALK_RECOG'},
            remapping = {'text':'recog_talk','timeout':'timeout_talk'}
        )

        smach.StateMachine.add('TALK_FRONT', TalkState.getInstance(),
            transitions = {'succeeded':'LISTEN_DIRECT_SPEECH','aborted':'TALK_FRONT'},
            remapping = {'text':'front_talk','timeout':'timeout_talk'}
        )

        smach.StateMachine.add('TALK_REAR', TalkState.getInstance(),
            transitions = {'succeeded':'succeeded','aborted':'TALK_REAR'},
            remapping = {'text':'rear_talk','timeout':'timeout_talk'}
        )                
        
        smach.StateMachine.add('TALK_MOVE_REAR', TalkState.getInstance(),
            transitions = {'succeeded':'TALK_REAR','aborted':'TALK_REAR'},
            remapping = {'text':'move_talk','timeout':'move_timeout_talk'}
        )

        smach.StateMachine.add('LISTEN_DIRECT_SPEECH',DirectSpeechRecognition.getInstance(),
            transitions={ 'succeeded':'TALK_MOVE_REAR' ,'aborted':'aborted'},
            remapping={'timeout':'timeout_direct_speech'}
        )

        # smach.StateMachine.add('LISTEN_INDIRECT_SPEECH',IndirectSpeechRecognition.getInstance(),
        #     transitions={ 'succeeded':'TALK_END','aborted':'aborted' },
        #     remapping={'timeout':'timeout_indirect_speech'}
        # )

        smach.StateMachine.add('TALK_END', TalkState.getInstance(),
            transitions = {'succeeded':'succeeded','aborted':'TALK_END'},
            remapping = {'text':'end_talk','timeout':'timeout_talk'}
        )  

        
    #sm.set_initial_state(['TALK_RECOG'])
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('speech_recognition_and_audio_detection')
    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('speech_recognition_and_audio_detection', sm, '/SPEECH_RECOGNITION_AND_AUDIO_DETECTION_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
