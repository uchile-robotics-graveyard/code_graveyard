#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf
import time
from bender_macros.speech import Talk
# services
from bender_srvs.srv import *
from bender_utils.ros import benpy

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class setup(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'],
			output_keys=['timeinit'])
	
	def execute(self, userdata):

		rospy.loginfo('Executing state: setup')
		Talk.getInstance("i am looking for some operator")

		userdata.timeinit = time.time()
		#rospy.sleep(1)
		return 'succeeded'

class FindFace(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['timeout','Face Detected','Face Not Detected','succeeded','aborted','preempted'],
		 input_keys=['time','timeinit'])
		
		self.face_detection = benpy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
		
		self.counterface = 0
		self.counternoface = 0
		self.count = 1

	def execute(self, userdata):
		rospy.loginfo('Executing state: FindFace')
		if userdata.time>0 and time.time() - userdata.time > userdata.timeinit:
			return 'timeout'
		
		req = FaceInfoRequest()
		req.use_image = False
		req.return_images = False
		try:
			resp = self.face_detection(req)
		except rospy.ServiceException, e:
			self.counterface=0
			return 'aborted'

		entro = False
		if resp.n_faces > 0 : 
			for i in range(0,resp.n_faces):
				print resp.BBoxes[i].height 
				if resp.BBoxes[i].height > 180:
					self.counterface+=1
					self.counternoface=0
					entro = True
		if entro == False:
			self.counterface=0
			self.counternoface+=1

		if self.counterface >= 2:
			self.counterface=0
			return 'Face Detected'
		if self.counternoface >= 3*self.count:
			self.count+=1
			self.counternoface=0
			return 'Face Not Detected'
		rospy.sleep(0.1)
		return 'succeeded'


class AskCome(smach.State):

	def __init__(self):
		smach.State.__init__(self,
							outcomes=['succeeded','aborted','preempted'],
							input_keys=['textintro'])
		
		self.idx = 0

	def execute(self, userdata):
		rospy.loginfo('Executing state AskCome...')
		
		text = ""
		if type(userdata.textintro) == str:
			text = userdata.textintro
		elif type(userdata.textintro) == list:
			lst_l = len(userdata.textintro)
			text = userdata.textintro[self.idx]
			self.idx = min(self.idx + 1, lst_l-1)
		else:
			return 'aborted'
		
		Talk.getInstance(text,len(text)/8)
		return 'succeeded'
	

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():
	
	sm = smach.StateMachine(
		outcomes=['Face Detected','Face Not Detected','succeeded','aborted','preempted'],
		input_keys=['time','textintro']
	)

	with sm:

		smach.StateMachine.add('SETUP',setup(),
			transitions = {'succeeded':'FindFace'},
		)
		smach.StateMachine.add('FindFace', FindFace(), 
			transitions={
				'Face Detected': 'Face Detected', 
				'timeout':'Face Not Detected',
				'Face Not Detected':'ASKCOME',
				'succeeded': 'FindFace',
				'aborted': 'Face Not Detected'}
		)
		smach.StateMachine.add('ASKCOME', AskCome(),
			transitions = {'succeeded':'FindFace'}
		)
	return sm

# main
if __name__ == '__main__':

	rospy.init_node('findface')

	sm = getInstance()

	#time = -1 para tiempo ilimitado
	#texintro = frase para que se acerquen
	ud = smach.UserData()
	ud.time = 20

	# introspection server
	sis = smach_ros.IntrospectionServer('findface', sm, '/FINDFACE_SM')
	sis.start()
	outcome = sm.execute(ud)
	sis.stop()
