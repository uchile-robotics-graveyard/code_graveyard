#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.speech import Talk  
from bender_utils.ros import benpy

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state WaitCommand
class WaitCommand(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.ready = False

        # Services
        self.start_reco_client = benpy.ServiceProxy('/bender/speech/recognizer/start', Empty)
        self.stop_reco_client  = benpy.ServiceProxy('/bender/speech/recognizer/stop', Empty)
        self.load_dict_client  = benpy.ServiceProxy('/bender/speech/recognizer/load_dictionary', load_dictionary_service)
        

    def recognitionCB(self,msg):

        if msg.data == 'bender go on':
            self.ready = True


    def execute(self, userdata):
        rospy.loginfo('Executing state WaitCommand')

        Talk.getInstance('please tell me ... bender go on',5)

        try:
            # load dictionary
            self.load_dict_client(dictionary="go_on")
            
            # start the recognition
            self.start_reco_client()

            
        except rospy.ServiceException, e:
            rospy.logerr("ups. failed to load or start speech recognition. " + str(e))
            return "aborted"

        # start result subscription
        rospy.Subscriber('/bender/speech/recognizer/output', std_msgs.msg.String, self.recognitionCB)

        while not self.ready and not rospy.is_shutdown():
            rospy.sleep(0.3);

        Talk.getInstance('okey, i will continue',3)
        
        try:
            # Stop recognition
            self.stop_reco_client()
        except rospy.ServiceException, e:
            rospy.logerr("ups. failed to stop speech recognition. " + str(e))

        self.ready = False
        return 'succeeded'


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:

        smach.StateMachine.add('WAIT_COMMAND', WaitCommand(),
                           transitions={'succeeded':'succeeded'})
    return sm


if __name__ == '__main__':

    rospy.init_node('wait_command_sm')
    
    rospy.logwarn("Starting Wait Command Test")
    sm = getInstance()
    
    # introspection server
    sis = smach_ros.IntrospectionServer('talk_sm', sm, '/TALK_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    
    rospy.logwarn("Wait Command Test Finished")
