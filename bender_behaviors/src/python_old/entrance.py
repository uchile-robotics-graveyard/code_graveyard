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
    global goal_reached
    goal_reached = (data.data == "Goal Reached")
    
def buttonCallback(data):
    global button_state
    if data.data:
        print "True weon TRUE"
    else:
        print "FALSE WEON"
    button_state=data.data
    

def main():
    
    global button_memory
    global button_state
    global goal_reached
    global DIST_OBJ

    rospy.init_node('entrance')    
    pub = rospy.Publisher('/cmd_vel', Twist)
    order = Twist()
    
    goalFB = rospy.Subscriber('/goalServer/feedback', String, goalFBcallback)
    buttonS = rospy.Subscriber('/stop_button', Bool, buttonCallback)
    
    setGoal=rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
    #rospy.wait_for_service('/goalServer/sendGoal')
    
    cancelGoal=rospy.ServiceProxy('/goalServer/cancelGoal', Empty)
    #rospy.wait_for_service('/goalServer/sendGoal')
    
    doorDetection=rospy.ServiceProxy('/door_open_detector/isopen', DoorDetector)
    
    lookAt=rospy.ServiceProxy('/goalServer/lookAtPose', lookToPose)
    #rospy.wait_for_service('/goalServer/sendGoal')
    lookPose=lookToPose()
    
    mapLoader = rospy.ServiceProxy('/semantic_map_server/load', save_load_Map)
    rospy.wait_for_service('/semantic_map_server/load')
    mapLoader("eindhoven.sem_map")
    
    semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
    rospy.wait_for_service('/semantic_map_server/load')
    resp=semCaller("start")
    
    initialPosePub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped)
    initialPose =PoseWithCovarianceStamped()
    

    #ver como setear la pose inicial
    initialPose.header.frame_id="/map"
    initialPose.pose.pose=resp.semObj.pose
    initialPose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    initialPosePub.publish(initialPose)
    
    #esperar un poco
    while not doorDetection.call(100).DoorState:
        Sleep(1)
    print "esperando un poco"
    time.sleep(3)
    
    #avanzar
    print "avanzando"
    order.linear.x=0.3
    order.angular.z=0.0
    pub.publish(order)

    
    #esperar
    print "esperando otro poco"
    time.sleep(4)
    
    #frenar
    print "frenando"
    order.linear.x=0.0
    order.angular.z=0.0
    pub.publish(order)
    
    #ir al rips point
    print "al rips point!"
    livingPose=semCaller("living room")
    setGoal(livingPose.semObj.pose,0)
    Sleep(0.5)
    
    #esperar a que llegue con boton de emegencia
    goal_reached=False
    while not goal_reached:
        if button_state:
            cancelGoal()
            while button_state:
                Sleep(0.5)
                cancelGoal()
            setGoal(livingPose.semObj.pose,0)
            Sleep(1)
        Sleep(0.5)

    
    
    #ir a la salida
    print "a la salida!"
    livingPose=semCaller("exit hall")
    setGoal(livingPose.semObj.pose,0)
   
     #esperar a que llegue con boton de emegencia
    goal_reached=False
    while not goal_reached:
        if button_state!=button_memory:
            button_memory=button_state
            if button_state==True:
                print "botton apretado"
            else:
                print "boton soltado"
        else:
            if not button_state:
                setGoal(livingPose.semObj.pose,0)
            else:
                cancelGoal()
        Sleep(0.5)
        

if __name__ == '__main__':
    main()
