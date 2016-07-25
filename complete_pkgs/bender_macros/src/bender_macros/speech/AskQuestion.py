#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from bender_macros.speech import Talk
from bender_srvs.srv import TimerString
from bender_macros.speech import TalkState
from std_msgs.msg import String


class ask(smach.State):
	
	def __init__(self,text,speech):
		smach.State.__init__(self, outcomes=['understand','time over','aborted'],
									input_keys=['timeout'])
		self.speech=speech
		self.attempt = 1
		self.text = text

	def execute(self, userdata):
		Talk.getInstance('Please ask me the question ' + str(self.text) + ' after the tone', 4)
		try:
			response = rospy.ServiceProxy('answer_request', TimerString)
			output = response(userdata.timeout,self.speech)
			if output.timeout == True:
				return 'time over'
			else:
				res = output.data
				inout = res.split(':')
				# Talk.getInstance('You asked '+inout[0],len(inout[0])/6)
				Talk.getInstance(inout[1],len(inout[1])/8)
				return 'understand'
		except rospy.ServiceException, e:
			print e
			return 'aborted'

if __name__ == '__main__':

	rospy.init_node('AskQuestion')

	sm = ask('','indirect')
	ud = smach.UserData()
	ud.timeout = 10
	outcome = sm.execute(ud)
