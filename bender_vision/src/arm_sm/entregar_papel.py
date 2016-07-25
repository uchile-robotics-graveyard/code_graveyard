#!/usr/bin/env python

import roslib; roslib.load_manifest('arm_sm')
import rospy
import time
import smach
import smach_ros
from bender_planning_old.srv import * 

class entregar_papel(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1'])

	def execute(self, userdata):
		rospy.wait_for_service('/right_arm/plan_state')
		rospy.wait_for_service('/right_arm/orientar_grip')
		rospy.wait_for_service('/right_arm/mover_grip_ang')
		rospy.wait_for_service('/right_arm/mover_muneca_ang')
		rospy.wait_for_service('/right_arm/torque_enable')
		rospy.wait_for_service('/right_arm/posicion_reposo')
		try:
			mover_muneca1 = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
			resp1 = mover_muneca1(-1.4,0.6)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(0.3)
		try:
			plan1 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
			resp1 = plan1(0.6,0.0,0.0,1.92)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(0.5)
		try:
			plan2 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
			resp2 = plan1(-0.4,0.0,0.0,1.7)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(0.6)
		try:
			orient_grip = rospy.ServiceProxy('/right_arm/orientar_grip', Dummy)
			resp3 = orient_grip()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		time.sleep(5)

		try:
			mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
			resp6 = mover_grip1(0.6,0.3)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(2)
		try:
			mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
			resp6 = mover_grip1(0.3,0.3)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(1)
		try:
			mover_muneca2 = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
			resp6 = mover_muneca2(-1.4,0.6)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(1)
		try:
			plan3 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
			resp7 = plan3(0.6,0.0,0.0,1.92)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		time.sleep(1)
		try:
			plan4 = rospy.ServiceProxy('/right_arm/posicion_reposo', Dummy)
			resp8 = plan4()
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
		smach.StateMachine.add('PAPEL', entregar_papel(),
													transitions={'outcome1':'outcome4'})

	# Execute SMACH plan
	outcome = sm.execute()



if __name__ == '__main__':
	main()
