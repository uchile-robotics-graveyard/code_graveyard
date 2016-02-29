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
speech_returned=False
speech_talking=False
speech_bad_detection=False

def speechSynthStatusCallback(data):
    global speech_talking
    speech_talking=(data.data=='Talking')

def speechRecognizerCallback(data):
    global speech_returned
    global speech_bad_detection
    print data
    if data.data=='bender leave the arena' or data.data=='bender leave':
        speech_returned=True
    else:
        speech_bad_detection=True
    return;

def goalFBcallback(data):
    global goal_reached
    goal_reached = (data.data == "Goal Reached")
    print data
    
    
def buttonCallback(data):
    global button_state
    button_state=data.data

def waitForOrder():
    speechRecoStart= rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop= rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    
    speechSubs=rospy.Subscriber('/speech_recognizer/output',String,speechRecognizerCallback)
    #wait for exit (bender leave the arena)
    global speech_returned
    global speech_bad_detection
    speechRecoStart()
    while not speech_returned:
        if speech_bad_detection :
            speechRecoStop()
            talk("I did not understand. Would you please repeat?")
            speechRecoStart()
            speech_bad_detection=False
        Sleep(0.5)
    speechRecoStop()

def talk(text):
    global speech_talking
    speechSynth= rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
    speechSynth(text)
    while not speech_talking:
        Sleep(0.5)
    while speech_talking:
        Sleep(0.5)
    
    
def waitForOrder():
    speechRecoStart= rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop= rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    
    speechSubs=rospy.Subscriber('/speech_recognizer/output',String,speechRecognizerCallback)
    #wait for exit (bender leave the arena)
    global speech_returned
    global speech_bad_detection
    speechRecoStart()
    while not speech_returned:
        if speech_bad_detection:
            speechRecoStop()
            speechRecoStart()
            talk("I did not understand. Would you please repeat?")
            speech_bad_detection=False
        Sleep(0.5)
    speechRecoStop()

def talk(text):
    global speech_talking
    speechSynth= rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
    speechSynth(text)
    while not speech_talking:
        Sleep(0.5)
    while speech_talking:
        Sleep(0.5)

def armRutine():
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
    Sleep(0.1)
    try:
        plan1 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp1 = plan1(0.6,0.0,0.0,1.92)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(0.1)
    try:
        plan2 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp2 = plan1(-0.4,0.0,0.0,1.7)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(0.1)
    try:
        orient_grip = rospy.ServiceProxy('/right_arm/orientar_grip', Dummy)
        resp3 = orient_grip()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    talk("Please take my registration form.")
    Sleep(5)

    try:
        mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
        resp6 = mover_grip1(0.5,0.3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(2)
    try:
        mover_grip1 = rospy.ServiceProxy('/right_arm/mover_grip_ang', AngVel)
        resp6 = mover_grip1(0.3,0.3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(0.1)
    try:
        mover_muneca2 = rospy.ServiceProxy('/right_arm/mover_muneca_ang', AngVel)
        resp6 = mover_muneca2(-1.4,0.6)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(0.1)
    try:
        plan3 = rospy.ServiceProxy('/right_arm/plan_state', PlanningGoalState)
        resp7 = plan3(0.6,0.0,0.0,1.92)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    Sleep(0.1)
    try:
        plan4 = rospy.ServiceProxy('/right_arm/posicion_reposo', Dummy)
        resp8 = plan4()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return

def goalWithStopButton(destination):
    global goal_reached
    global button_state
    
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

def setInitialPose(pose="start"):
        
    semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
    rospy.wait_for_service('/semantic_map_server/load')
    resp=semCaller(pose)
    
    initialPosePub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped)
    initialPose =PoseWithCovarianceStamped()
    
    initialPose.header.frame_id="/map"
    initialPose.header.stamp = rospy.Time.now()
    initialPose.pose.pose=resp.semObj.pose
    initialPose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    initialPosePub.publish(initialPose)

def waitForOpenDoor(distance=100):
    global doorDetection
    doorDetection=rospy.ServiceProxy('/door_open_detector/isopen', DoorDetector)
    print "esperando detector de puerta abierta"
    rospy.wait_for_service('/door_open_detector/isopen')
    
    print "esperando puerta abierta"
    while not doorDetection.call(distance).DoorState:
        Sleep(1)

def main():

    
    rospy.init_node('rips')
    
    pub = rospy.Publisher('/cmd_vel', Twist)
    order = Twist()
    
    goalFB = rospy.Subscriber('/goalServer/feedback', String, goalFBcallback)
    buttonS = rospy.Subscriber('/stop_button', Bool, buttonCallback)
    
    mapLoader = rospy.ServiceProxy('/semantic_map_server/load', save_load_Map)
    rospy.wait_for_service('/semantic_map_server/load')
    mapLoader("eindhoven.sem_map")

    dictionaryLoader = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    rospy.wait_for_service('/speech_recognizer/load_dictionary')
    dictionaryLoader("rips")
    
    speechSynthState=rospy.Subscriber('/speech_synthesizer/status',String,speechSynthStatusCallback)
    
    #Pose inicial
    talk("Setting initial pose")
    setInitialPose("start")
    #setInitialPose("test ini")
    
    Sleep(2)

    #Esperando puerta abierta
    talk("waiting for open door")
    waitForOpenDoor()

    #esperar un poco
    talk("Entering the arena")
    arenaEntrance(5)
    #arenaEntrance(1)
    
    #Sleep(5)
    
    #ir al rips point
    talk("Navigating")
    goalWithStopButton("rips point")
    #goalWithStopButton("test rips")
    
    Sleep(5)

    #rutina
    talk("Hello. My name is Bender. I am the homebreakers robot. I want to give you the registration board.")
    armRutine()
    talk('Thank you.')
    
    waitForOrder()
    talk('Okay, I am leaving now. See you tomorrow.')

    #ir a la salida
    print "a la salida!"

    goalWithStopButton("exit ini")
    goalWithStopButton("exit end")
    #goalWithStopButton("test end")
    #Sleep(3)
    talk("hasta la vista. baby")

    

if __name__ == '__main__':
    main()
