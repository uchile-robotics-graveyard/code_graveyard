#!/usr/bin/env python

import roslib; roslib.load_manifest('arm_sm')
import rospy
import time
import smach
import smach_ros
from bender_planning_old.srv import * 
from arm_vision_interface.srv import * 
from PlaneDetector.srv import * 
from SiftDetector.srv import * 

peticion = "mover"        

class detectar_manipular(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

        rospy.wait_for_service('/right_arm/plan_state')
        rospy.wait_for_service('/right_arm/orientar_grip')
        rospy.wait_for_service('/right_arm/mover_grip_ang')
        rospy.wait_for_service('/right_arm/mover_muneca_ang')
        rospy.wait_for_service('/right_arm/torque_enable')
        rospy.wait_for_service('/right_arm/posicion_reposo')
        rospy.wait_for_service('/right_arm/posicion_inicial')
        rospy.wait_for_service('/right_arm/posicion_postmanipulacion1')
        rospy.wait_for_service('/right_arm/posicion_premanipulacion1')
        rospy.wait_for_service('/right_arm/posicion_premanipulacion2')
        rospy.wait_for_service('/right_arm/abrir_grip')
        rospy.wait_for_service('/right_arm/cerrar_grip')
        rospy.wait_for_service('/right_arm/plan_cartesian')
        rospy.wait_for_service('/right_arm/grasp')

        rospy.wait_for_service('/siftOn')
        rospy.wait_for_service('/planeOn')
        rospy.wait_for_service('/arm_vision_interface/detect_obj')
        rospy.wait_for_service('/arm_vision_interface/reset_obj')

    def execute(self, userdata):
        detected=False
        x = 48
        y = 10
        z = 95
        try:
            detect = rospy.ServiceProxy('/arm_vision_interface/detect_obj', ObjectDetection)
            detected_obj = detect()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            pre1 = rospy.ServiceProxy('/right_arm/posicion_premanipulacion1', Dummy)
            resp3 = pre1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            pre2 = rospy.ServiceProxy('/right_arm/posicion_premanipulacion2', Dummy)
            resp4 = pre2()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # try:
        #     abrir = rospy.ServiceProxy('/right_arm/abrir_grip', Dummy)
        #     resp4 = abrir()
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e
        try:
            plan1 = rospy.ServiceProxy('/right_arm/grasp', PlanningGoalCartesian)
            resp5 = plan1(x,y,z)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        try:
            orient_grip = rospy.ServiceProxy('/right_arm/orientar_grip', Dummy)
            resp6 = orient_grip()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        # try:
        #     cerrar = rospy.ServiceProxy('/right_arm/cerrar_grip', LoadMode)
        #     resp7 = cerrar(0)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e
        try:
            post1 = rospy.ServiceProxy('/right_arm/posicion_postmanipulacion1', Dummy)
            resp8 = post1()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        try:
            inicial = rospy.ServiceProxy('/right_arm/posicion_inicial', Dummy)
            resp9 = inicial()
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
                    transitions={    'outcome1':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
