#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_utils.ros import benpy

# services
import bender_srvs.srv

class TalkState(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['text','timeout']
        )
        self.talk_client = benpy.ServiceProxy('/bender/speech/synthesizer/synthesize',bender_srvs.srv.synthesize)
        
    def execute(self, userdata):

        try:
            self.talk_client.wait_for_service()
            self.talk_client(userdata.text)
    
            if userdata.timeout < 0.0:
                # handle timeout ourselves
                pass
    
            elif userdata.timeout > 0.0:
                rospy.sleep(userdata.timeout)
            return 'succeeded'
        except rospy.ServiceException, e:
            return 'aborted'

def getInstance(text='',timeout=0):

    if text=='' and timeout==0:
        
        sm = smach.StateMachine(
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['text','timeout']
        )

        with sm:
            smach.StateMachine.add('TALK', TalkState())

        return sm

    else:

        sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        sm.userdata.text = text
        sm.userdata.timeout = timeout
        with sm:
            smach.StateMachine.add('TALK', TalkState())

        return sm

    pass


if __name__ == '__main__':

    import sys
    test_no = 1
    if len(sys.argv) == 2:
        test_no = int(sys.argv[1])

    rospy.init_node('talk_state')
    
    ud = smach.UserData()
    ud.text = ''
    ud.timeout = 0
    sm = getInstance()

    # test
    if test_no == 1:
        ud.text = 'hello, i am bender, and this is the first test'
        ud.timeout = 2.0
        print "test1"

    elif test_no == 2:
        sm = getInstance('hello, i am bender and this is test two', 2.0)
        print "test2"


    # introspection server
    sis = smach_ros.IntrospectionServer('talk_sm', sm, '/TALK_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()