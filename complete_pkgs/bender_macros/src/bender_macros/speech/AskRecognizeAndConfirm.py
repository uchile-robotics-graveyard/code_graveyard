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
from bender_macros.speech import AskForConfirmation
from bender_macros.speech import Recognize
from bender_macros.speech import AskForDrink

class Setup(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Setup')
		rospy.sleep(0.1)
		return 'succeeded'

class AskQuestion(smach.State):

	def __init__(self):
		smach.State.__init__(self,
							outcomes=['succeeded','aborted','preempted'],
							input_keys=['question','timeout','dictionary']
							)

	def execute(self, userdata):
		rospy.loginfo('Executing state AskQuestion... Talking')
		
		Talk.getInstance(userdata.question,userdata.timeout)
		return 'succeeded'

      
def getInstance():

	sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
							input_keys=['question','timeout','dictionary'],
							output_keys=['answer']
							)


	with sm:

		smach.StateMachine.add('SETUP',Setup(),
						   transitions={'succeeded':'ASK_QUESTION'}
						   )
		smach.StateMachine.add('ASK_QUESTION',AskQuestion(),
						   transitions={'succeeded':'RECOGNIZE'},
						   remapping={'question':'question',
						   			'timeout':'timeout'}
						   )
		smach.StateMachine.add('RECOGNIZE',Recognize.getInstance(),
						   transitions={'succeeded':'CONFIRM'},
						   remapping={'dictionary':'dictionary',
						   			'recognized_word':'answer'}
						   )
		smach.StateMachine.add('CONFIRM',AskForConfirmation.getInstance(),
						   transitions={'yes':'succeeded',
						   				'no':'ASK_QUESTION'},
						   remapping={'text':'answer',
						   			'timeout':'timeout'}
						   )
						   
	return sm


if __name__ == '__main__':

	rospy.init_node('AskRecognizeAndConfirm')

	sm = getInstance()
	ud = smach.UserData()

	ud.question = "What's your command"
	ud.timeout = 3
	ud.dictionary = 'gpsr'

	# introspection server
	sis = smach_ros.IntrospectionServer('AskRecognizeAndConfirm', sm, '/ASK_RECOGNIZE_AND_CONFIRM_SM')
	sis.start()
	outcome = sm.execute(ud)
	sis.stop()
