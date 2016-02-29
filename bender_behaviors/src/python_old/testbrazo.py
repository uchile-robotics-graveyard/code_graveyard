#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import math
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
from bender_msgs.msg import *
from bender_srvs.srv import *


goal_reached=False
button_state=False
button_memory=False
DIST_OBJ = 1500

def goalFBcallback(data):
    goal_reached = (data.data == "Goal Reached")
    
def buttonCallback(data):
    button_state=data.data
    
def armRutine():
    rospy.wait_for_service('/right_arm/plan_state')
    rospy.wait_for_service('/right_arm/orientar_grip')
    rospy.wait_for_service('/right_arm/mover_grip_ang')
    rospy.wait_for_service('/right_arm/mover_muneca_ang')
    rospy.wait_for_service('/right_arm/torque_enable')
    try:
        mover_muneca1 = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
        resp1 = mover_muneca1(-1.4,0.6)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        plan1 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp1 = plan1(0.6,0.0,0.0,1.92)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        plan2 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp2 = plan1(-0.4,0.0,0.0,1.7)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        orient_grip = rospy.ServiceProxy('/right_arm/orientar_grip', Dummy)
        resp3 = orient_grip()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    time.sleep(5)
    #///////////////////////HABLAR/////////////////////////////////////

    try:
        mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
        resp6 = mover_grip1(0.5,0.3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    time.sleep(2)
    try:
        mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
        resp6 = mover_grip1(0.3,0.3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        mover_muneca2 = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
        resp6 = mover_muneca2(-1.4,0.6)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        plan3 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp7 = plan3(0.6,0.0,0.0,1.92)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        plan4 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp8 = plan4(0.0,0.0,0.0,0.0)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        alinear_muneca = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
        resp9 = alinear_muneca(0.0,0.6)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    try:
        torque_off = rospy.ServiceProxy('/right_arm/torque_enable', TorqueEnable)
        resp10 = torque_off("",False)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def goalWithStopButton(destination):
    #init
    semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
    rospy.wait_for_service('/semantic_map_server/load')
    setGoal=rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
    rospy.wait_for_service('/goalServer/sendGoal')
    
    print "al !"+destination
    livingPose=semCaller(destination)
    setGoal(livingPose.semObj.pose,0)
    Sleep(0.5)
    
    #esperar a que llegue con boton de emegencia
    goal_reached=False
    while not goal_reached:
        if button_state:
            cancelGoal()
            while button_state:
                Sleep(0.5)
            setGoal(livingPose.semObj.pose,0)
            Sleep(1)
        Sleep(0.5)
    return

def arenaEntrance(duration=5):
    
    pub = rospy.Publisher('/cmd_vel', Twist)
    order = Twist()
    
    print "esperando un poco"
    time.sleep(5)
    
    #avanzar
    print "avanzando"
    order.linear.x=0.3
    order.angular.z=0.0
    pub.publish(order)
    
    #esperar
    print "esperando otro poco"
    time.sleep(duration)
    
    #frenar
    print "frenando"
    order.linear.x=0.0
    order.angular.z=0.0
    pub.publish(order)
    return

def setInitialPose():
    semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
    rospy.wait_for_service('/semantic_map_server/load')
    resp=semCaller("start")
    
    initialPose.header.frame_id="/map"
    initialPose.pose.pose=resp.semObj.pose
    initialPose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    initialPosePub.publish(initialPose)

def waitForOpenDoor(distance=100):
    doorDetection=rospy.ServiceProxy('/door_open_detector/isopen', DoorDetector)
    print "esperando detector de puerta abierta"
    rospy.wait_for_service('/door_open_detector/isopen')
    
    print "esperando puerta abierta"
    while not doorDetection.call(distance).DoorState:
        Sleep(1)
    time.sleep(3)

def main():
    armRutine()
    rospy.init_node('entrance')    
    pub = rospy.Publisher('/cmd_vel', Twist)
    order = Twist()
    
    goalFB = rospy.Subscriber('/goalServer/feedback', String, goalFBcallback)
    buttonS = rospy.Subscriber('/stop_button', Bool, buttonCallback)
    
    mapLoader = rospy.ServiceProxy('/semantic_map_server/load', save_load_Map)
    rospy.wait_for_service('/semantic_map_server/load')
    mapLoader("eindhoven.sem_map")
    
    initialPosePub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped)
    initialPose =PoseWithCovarianceStamped()
    
    #ver como setear la pose inicial
    setInitialPose()
    
    #esperar un poco
    arenaEntrance()
    
    #ir al rips point
    goalWithStopButton("rips point")
    
    #rutina
    armRutine()
    
    #ir a la salida
    print "a la salida!"
    goalWithStopButton("end_ini")
    

if __name__ == '__main__':
    main()
