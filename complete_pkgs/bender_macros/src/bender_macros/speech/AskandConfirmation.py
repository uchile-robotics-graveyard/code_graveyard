#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.speech import Recognize
from bender_macros.speech import Talk

class Confirmation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','fail'],
                                    input_keys=['conf']
                            )

    def execute(self, userdata):
        print userdata.conf
        if userdata.conf == 'bender yes':
            return 'succeeded'
        if userdata.conf == 'bender no':
            return 'fail'
        rospy.sleep(0.5)
        return 'aborted'

class talk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['text','timeout']
                            )

    def execute(self, userdata):
        Talk.getInstance(userdata.text,userdata.timeout)
        return 'succeeded'

class talk1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['answer','timeout']
                            )

    def execute(self, userdata):
        Talk.getInstance("did you say? " + userdata.answer,userdata.timeout)
        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['text','timeout','dic_ask']
                            )

    sm.userdata.dic_conf = 'confirmation'

    with sm:
        smach.StateMachine.add('TALK',talk(),
                           transitions={'succeeded':'RECOGNIZER'},
                           remapping={'text':'text', 'timeout':'timeout'}
                           )
        smach.StateMachine.add('RECOGNIZER',Recognize.getInstance(),
                           transitions={'succeeded':'TALK1'},
                           remapping={'dictionary':'dic_ask',
                                    'recognized_word':'answer'}
                           )
        smach.StateMachine.add('TALK1',talk1(),
                           transitions={'succeeded':'CONFIRM_RECOGNIZER'},
                           remapping={'text':'text', 'timeout':'timeout'}
                           )
        smach.StateMachine.add('CONFIRM_RECOGNIZER',Recognize.getInstance(),
                           transitions={'succeeded':'CONFIRM'},
                           remapping={'dictionary':'dic_conf',
                                    'recognized_word':'conf'}
                           )
        smach.StateMachine.add('CONFIRM',Confirmation(),
                           transitions={'fail':'TALK',
                                        'succeeded':'succeeded',
                                        'aborted':'CONFIRM_RECOGNIZER'},
                           remapping={'conf':'conf'}
                           )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('askandconfirmation')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.text = 'What is your name'
    ud.timeout = 4
    ud.dic_ask = 'name'

    # introspection server
    sis = smach_ros.IntrospectionServer('askandconfirmation', sm, '/S_ASKCONFIRMATION_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
