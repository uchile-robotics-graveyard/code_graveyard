#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_vision')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import * 


peticion = "fanta"		

class detectar_manipular(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

		rospy.wait_for_service('/left_arm/plan_state')
		rospy.wait_for_service('/left_arm/orientar_grip')
		rospy.wait_for_service('/left_arm/mover_grip_ang')
		rospy.wait_for_service('/left_arm/mover_muneca_ang')
		rospy.wait_for_service('/left_arm/torque_enable')
		rospy.wait_for_service('/left_arm/posicion_reposo')
		rospy.wait_for_service('/left_arm/posicion_inicial')
		rospy.wait_for_service('/left_arm/posicion_postmanipulacion1')
		rospy.wait_for_service('/left_arm/posicion_premanipulacion1')
		rospy.wait_for_service('/left_arm/posicion_premanipulacion2')
		rospy.wait_for_service('/left_arm/abrir_grip')
		rospy.wait_for_service('/left_arm/cerrar_grip')
		rospy.wait_for_service('/left_arm/plan_cartesian')
		rospy.wait_for_service('/left_arm/grasp')

		rospy.wait_for_service('/siftOn')
		rospy.wait_for_service('/planeOn')
		rospy.wait_for_service('/arm_vision_interface/detect_obj')
		rospy.wait_for_service('/arm_vision_interface/reset_obj')

	def execute(self, userdata):
		detected=False
		try:
			planeon = rospy.ServiceProxy('/planeOn', planeOn)
			resp1 = planeon(0)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		try:
			sifton = rospy.ServiceProxy('/siftOn', siftOn)
			resp2 = sifton()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		try:
			reset = rospy.ServiceProxy('/arm_vision_interface/reset_obj', Dummy)
			reset_resp = reset()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(10)
		try:
			detect = rospy.ServiceProxy('/arm_vision_interface/detect_obj', ObjectDetection)
			detected_obj = detect()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		for i in range(len(detected_obj.name)):
			print detected_obj.name
			print detected_obj.x
			print detected_obj.y
			print detected_obj.z
			if detected_obj.name[i]==peticion:
				x=detected_obj.x[i]
				y=detected_obj.y[i]
				z=detected_obj.z[i]
				detected=True
		if detected:
			try:
				pre1 = rospy.ServiceProxy('/left_arm/posicion_premanipulacion1', Dummy)
				resp3 = pre1()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			try:
				pre2 = rospy.ServiceProxy('/left_arm/posicion_premanipulacion2', Dummy)
				resp4 = pre2()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				abrir = rospy.ServiceProxy('/left_arm/abrir_grip', Dummy)
				resp4 = abrir()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				plan1 = rospy.ServiceProxy('/left_arm/grasp', PlanningGoalCartesian)
				resp5 = plan1(x,y,z)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				orient_grip = rospy.ServiceProxy('/left_arm/orientar_grip', Dummy)
				resp6 = orient_grip()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				cerrar = rospy.ServiceProxy('/left_arm/cerrar_grip', LoadMode)
				resp7 = cerrar(0)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				post1 = rospy.ServiceProxy('/left_arm/posicion_postmanipulacion1', Dummy)
				resp8 = post1()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				inicial = rospy.ServiceProxy('/left_arm/posicion_inicial', Dummy)
				resp9 = inicial()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

		try:
			planeoff = rospy.ServiceProxy('/planeOff', planeOff)
			resp1 = planeoff(0)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		try:
			siftoff = rospy.ServiceProxy('/siftOff', siftOff)
			resp1 = siftoff(0)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		return 'outcome1'


def main():
	rospy.init_node('smach_example_state_machine')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['outcome4'])

	# Open the container
	with sm:
		# Add states to the container

		smach.StateMachine.add('DETECTAR_MANIPULAR', detectar_manipular(),
					transitions={	'outcome1':'outcome4'})

	# Execute SMACH plan
	outcome = sm.execute()



if __name__ == '__main__':
	main()
