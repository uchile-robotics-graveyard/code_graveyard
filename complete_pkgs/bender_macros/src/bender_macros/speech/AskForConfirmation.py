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

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','yes','no'],
                            input_keys=['text','timeout']
                            )

    sm.userdata.dictionary = 'confirmation'

    with sm:
        smach.StateMachine.add('TALK',talk(),
                           transitions={'succeeded':'CALL_RECOGNIZER'},
                           remapping={'text':'text', 'timeout':'timeout'}
                           )
        smach.StateMachine.add('CALL_RECOGNIZER',Recognize.getInstance(),
                           transitions={'succeeded':'CONFIRM'},
                           remapping={'dictionary':'dictionary',
                                    'recognized_word':'conf'}
                           )
        smach.StateMachine.add('CONFIRM',Confirmation(),
                           transitions={'fail':'no',
                                        'succeeded':'yes',
                                        'aborted':'CALL_RECOGNIZER'},
                           remapping={'conf':'conf'}
                           )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('confirmation')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.text = 'ASDF ASDF'
    ud.timeout = 4

    # introspection server
    sis = smach_ros.IntrospectionServer('confirmation', sm, '/S_CONFIRMATION_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
