#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *
from geometry_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_macros.nav import GoToPlace
from bender_macros.nav import ApproachToTable
from bender_macros.vision import PositionAndDetect
#from bender_behaviors.Stage1.GPSR import Grasp
from bender_macros.vision import FindObject
from bender_macros.speech import Talk
from bender_arm_control.arm_commander import Limb
from bender_macros.head import MoveAsus

from bender_macros.arm import GraspCartesian_table
from bender_macros.arm import GraspCartesian_shelf
from tf import transformations

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

	def execute(self, userdata):
		Talk.getInstance('I will try to grasp', 3)
		rospy.loginfo('Executing state Setup')


		face=Emotion()
		face.Order = ""
		face.Action = "MoveX"
		face.X = 0
		face_pub = rospy.Publisher('"bender/face/head',Emotion, queue_size=1)

		face_pub.publish(face)
				   


		rospy.sleep(0.5)
		return 'succeeded'



# define state ChooseObject
class ChooseObject(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','canceled'],
			input_keys=['object_name'],
			 io_keys=['object_position','object_counter','type,','height','radius'])
		self.transformer = rospy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)

	def execute(self, userdata):
		
		rospy.loginfo('Executing state ChooseObject')
		userdata.object_counter +=1
		if userdata.object_counter > 2:
			userdata.object_counter = 0
			return 'canceled'


		userdata.object_position = self.transfor(userdata.object_position)
		userdata.object_position.pose.orientation = Limb.simple_orientation
		userdata.object_position.header.stamp = rospy.Time.now()
		print "OBJECT"
		print userdata.object_position

		t,h,r = self.get_inf(userdata.object_name)
		print h
		print r
		userdata.type = t
		userdata.height = h
		userdata.radius = r
		return 'succeeded'
		  

	def transfor(self, pose):

		req = TransformerRequest()

		req.pose_in = pose#
		req.frame_out = "bender/base_link"
		req.pose_in.header.stamp = rospy.Time.now()
		# req.pose_in.header.frame_id = "bender/sensors/rgbd_head_rgb_optical_frame"


		transf_out = self.transformer(req)


		# transf_out.pose_out.pose.position.x +=  -0.06
		# transf_out.pose_out.pose.position.y +=  -0.015
		# transf_out.pose_out.pose.position.z +=  0.05

		q = transformations.quaternion_from_euler(1.54,1.54,1.54)

		transf_out.pose_out.pose.orientation.x =  q[0]
		transf_out.pose_out.pose.orientation.y =  q[1]
		transf_out.pose_out.pose.orientation.z =  q[2]
		transf_out.pose_out.pose.orientation.w =  q[3]

		return transf_out.pose_out

	def get_inf(self,name_obj):
		h = 0.1
		r = 0.05    
		try:
			name = '/ParseOrder/'+name_obj  
			inf = rospy.get_param(name)
			t = inf[0]
			h = inf[1] - 0.06
			r = inf[2]
		except:
			print "dont find information object"

		return h,r


# define state Apologize
class Apologize(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
							 output_keys=['report'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Apologize')

		Talk.getInstance('I am sorry, I could not find any objects.', 4)
		userdata.report = "I am sorry, I could not find the object."

		return 'succeeded'


# define state Apologize2
class Apologize2(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
							 output_keys=['report'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Apologize2')

		Talk.getInstance('I am sorry, I can not reach the detected objects.', 4)
		userdata.report = "I am sorry, I can not reach the detected object."

		return 'succeeded'

def getInstance():

	sm = smach.StateMachine(
		outcomes=['succeeded','notGrabb','aborted','preempted'],
		input_keys=['object_name'],
		output_keys=['selected_arm','report'])
	
	
	sm.userdata.object_counter = 0
	sm.userdata.object_position = PoseStamped()
	sm.userdata.object_id = []
	sm.userdata.object_type = []
	sm.userdata.object_status = []
	sm.userdata.object_class_location = ""
	#sm.userdata.selected_position = geometry_msgs.msg.PoseStamped()
	sm.userdata.selected_type = ""
	sm.userdata.selected_arm = "/right_arm"
	sm.userdata.report = "I take the object"
	sm.userdata.approx_ik =True

	sm.userdata.deg_angle = 55
	sm.userdata.type = "CYLINDER"
	sm.userdata.height = 0.1
	sm.userdata.radius = 0.05


	with sm:

		smach.StateMachine.add('SETUP',Setup(),
			   transitions={'succeeded':'MOVE ASUS'})


		smach.StateMachine.add('MOVE ASUS',MoveAsus.getInstance(),
			   transitions={'succeeded':'POSITION_AND_DETECT'},
			   remapping={'deg_angle':'deg_angle'})

		smach.StateMachine.add('POSITION_AND_DETECT',FindObject.getInstance(),
			   transitions={'succeeded':'CHOOSE_OBJECT',
							'not_found':'APOLOGIZE'},
			   remapping={'object_name':'object_name',
						  'object_pose':'object_position',
						  'report':'object_id'}
						  )

		smach.StateMachine.add('CHOOSE_OBJECT',ChooseObject(),
			   transitions={'succeeded':'GRASP_OBJECT',
							'preempted' : 'POSITION_AND_DETECT',
							'canceled':'APOLOGIZE2'}
							)
		smach.StateMachine.add('GRASP_OBJECT',GraspCartesian_shelf.getInstance(),
			   transitions={'notgrabbing':'POSITION_AND_DETECT'},
			   remapping={'pose':'object_position',
						'height':'height',
						'radius': 'radius'})
		# smach.StateMachine.add  ('GRASP_OBJECT' , GraspObject_old.getInstance(),
		#       transitions  =  {'succeeded':'succeeded',
		#                    'notGrabbed':'POSITION_AND_DETECT'},
		#        remapping={'position':'object_position'}
		#           )
		smach.StateMachine.add('APOLOGIZE',Apologize(),
			   transitions={'succeeded':'notGrabb'})

		smach.StateMachine.add('APOLOGIZE2',Apologize2(),
			   transitions={'succeeded':'notGrabb'})
	return sm

# main
if __name__ == '__main__':

	rospy.init_node('pick')

	sm = getInstance()
	ud = smach.UserData()
	ud.object_name = "pringles"
	# introspection server
	sis = smach_ros.IntrospectionServer('pick', sm, '/PICK_SM')
	sis.start()
	outcome = sm.execute(ud)
	sis.stop()
