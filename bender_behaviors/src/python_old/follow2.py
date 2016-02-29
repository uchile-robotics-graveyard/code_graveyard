#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *
import math
import time

#variables globales
is_init=False
speech_talking=False
instruction_recognized=True
speech_bad_detection=False
instruction_to_recognize=""

follow_ID=-1
follow_R = -1
follow_Theta = -1

path_blocked=False
lost=False

oclusion_ID = -1
oclusion_R = -1
oclusion_Theta = 0
oclusion_Position = -1
oclusion_Direction = -1
blocking_ind = -1

set_follow_ID = -1


pub = rospy.Publisher('/cmd_vel', Twist)
twist = Twist()

#parametros globales
Pt = -0.0007 # Tralacion
Pr = 0.02   # Rotacion
follow_distace = 1500
lin_max = 0.7
thetamax = 0.7#1.0
path_blocked_angle_threshold = 20
lost_ID_angle = 15
lost_ID_distance=500
histeresis_threshold_linear=0.1
histeresis_threshold_theta=0.1
recuperation_step=0.1

debug=True

#callbacks
def talking_callback(status):
    global speech_talking
    if status.data == "Talking":
        speech_talking = True
    else:
        speech_talking = False

def speechRecognizerCallback(data):
    global instruction_recognized
    global speech_bad_detection
    global instruction_to_recognize
    print data.data
    if data.data==instruction_to_recognize:
        instruction_recognized=True
    else:
        speech_bad_detection=True
    return;

def estimation_callback(data):
    global follow_ID,follow_R,follow_Theta,is_init,lost
    
    if not is_init:
        return
    index=-1
    for i in range(len(data.ID)):
        if data.ID[i] == follow_ID:
            index=i
            break
    if index > -1:
        if path_blocked:
            lost = True
            debug_print("estimation_callback: no esta, path blocked --> lost")
            return

        follow_R=data.Distance[index]
        follow_Theta=data.Theta[index]
        lost=False
    else:
#         min_dist=9999999999999999
#         min_ind=-1
#         for i in range(len(data.ID)):
#             dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[i]*math.cos(data.Theta[i]*math.pi/180)
#             dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[i]*math.sin(data.Theta[i]*math.pi/180)
#             dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
#             if dist < min_dist:
#                 min_dist = dist
#                 min_ind = i
#                   
#         if min_dist < lost_ID_distance:
#             follow_ID=data.ID[min_ind]
#             follow_R=data.Distance[min_ind]
#             follow_Theta=data.Theta[min_ind]
#             debug_print("estimation_callback: no esta, alguien cerca,recovery succes --> follow ID: "+str(follow_ID))
#             set_follow_ID(follow_ID)
#             lost=False
#         else:
#             debug_print("estimation_callback: no esta,nadie esta cerca,recovery failure --> lost")
#             lost=True
        lost=True
    
    #si esta
        #si esta cerca
            #mantener id
        #else
            #buscar cercano
            #si esta lejos
                #perdido con detalles
            #else
                #cambiar id al mas cercano
    #else
        #buscar cercano
            #si esta lejos
                #perdido con detalles
            #else
                #cambiar id al mas cercan

def oclusion_callback(data):
    global follow_ID,follow_R,follow_Theta,oclusion_ID,oclusion_R,oclusion_Theta,oclusion_Position,oclusion_Direction,blocking_ind,path_blocked
    if not is_init:
        return
    oclusion_ID = data.ID
    oclusion_R = data.dist
    oclusion_Theta = data.theta
    oclusion_Position = data.position
    oclusion_Direction = data.direction
 
    for i in range(len(oclusion_ID)):
        if abs(oclusion_Theta[i]-follow_Theta)<path_blocked_angle_threshold:
            path_blocked = True
            bloking_ind = i
            debug_print("oclusion_callback: path blocked by index: "+str(bloking_ind))
        else: 
            path_blocked = False
            blocking_ind = -1
             
    if len(oclusion_ID)==0:
        path_blocked = False
        blocking_ind = -1

def init_callback(data):
    global follow_ID,follow_R,follow_Theta,is_init
    if not instruction_recognized:
        return
    if is_init:
        return
    for i in range(len(data.ID)):
        if abs(data.Distance[i]-follow_distace) < 300 and abs(data.Theta[i]) < 7 :
            follow_ID=data.ID[i]
            follow_R=follow_distace
            follow_Theta=0
            set_follow_ID(follow_ID)
            is_init=True
#funciones utiles
def talk(text):
    global speech_talking
    speechSynth= rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
    speechSynth(text)
    debug_print(text)
    speech_talking=False
    while not speech_talking:
        time.sleep(0.5)
    while speech_talking:
        time.sleep(0.5)

def move(forward,rotation):
    global twist
    twist.linear.x = forward
    twist.linear.x = min(twist.linear.x,lin_max)
    twist.linear.x = max(twist.linear.x,-lin_max)
     
    twist.angular.z = rotation
    twist.angular.z=min(twist.angular.z,thetamax)
    twist.angular.z=max(twist.angular.z,-thetamax)
     
#     if twist.linear.x<histeresis_threshold_linear:
#         twist.linear.x =0
#      
#     if twist.angular.z<histeresis_threshold_theta:
#         twist.angular.z = 0

    pub.publish(twist)

def waitForOrder(order):
    speechRecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    
    #wait for exit (bender leave the arena)
    global instruction_recognized
    global speech_bad_detection
    global instruction_to_recognize
    
    instruction_to_recognize=order
    
    speechRecoStart()
    debug_print("recognition started")
    
    count = 0
    while not instruction_recognized:
        if speech_bad_detection:
            speechRecoStop()
            talk("I did not understand. Would you please repeat?")
            speechRecoStart()
            speech_bad_detection=False
            count = count+1
            if count>1:
                instruction_recognized = True
                break
        time.sleep(1)
    speechRecoStop()

def debug_print(text):
    if debug:
        print text
    else:
        return

#estados
def waitingInstruction():
    waitForOrder("bender follow me")
    return

def InitFollow():
    global is_init,following
    while not is_init:
        talk("Please stand one point five meters in front of me.")
        time.sleep(3)
    talk("Ok, I will start following you now.")

def follow():
    global follow_R,follow_Theta,oclusion_ID,oclusion_R,oclusion_Theta,oclusion_Position,oclusion_Direction,twist
    
    P_counter=0
    follow_vel_anterior=0
    counter=0
    following=True
    while following:
        if path_blocked:
            if counter==0:
                talk("please do not block my path")
            move(0,0)
            twist = Twist()
            pub.publish(twist)
            follow_vel_anterior=0
            counter=counter+1
            debug_print("no me bloquees!")
        elif lost:
            debug_print("lost!")
#             if follow_Theta>lost_ID_angle:
#                 if twist.angular.z < histeresis_threshold_angular or twist.angular.z > -histeresis_threshold_angular:
#                     twist.angular.z = 0
#                     move(lin_max,twist.angular.z)
#                 else:
#                     twist.angular.z = twist.angular.z -recuperation_step
#                     move(lin_max,twist.angular.z)
#             elif follow_Theta<-lost_ID_angle:
#                 if twist.angular.z < histeresis_threshold_angular or twist.angular.z > -histeresis_threshold_angular:
#                     twist.angular.z = 0
#                     move(lin_max,twist.angular.z)
#                 else:
#                     twist.angular.z = twist.angular.z +recuperation_step
#                     move(lin_max,twist.angular.z)
#             else:
            move(lin_max,0)
            follow_vel_anterior=lin_max
        else:
            debug_print("siguiendo")
            counter=0
            move(Pt*(follow_distace-follow_R),Pr*follow_Theta)
#             if(abs(follow_vel_anterior-Pt*(follow_distace-follow_R))<0.2:
#                P_counter=P_counter+1
#             else:
#                 P_counter=0   
#             if P_counter>100:
#                 move(Pt*(follow_distace-follow_R)/3,Pr*follow_Theta)
#             else:
#                 move(Pt*(follow_distace-follow_R),Pr*follow_Theta)
                
    return

def main():
    global set_follow_ID,pub,twist
    
    rospy.init_node('follow')
    
    #servicions y topicos de speech
    rospy.wait_for_service('/speech_recognizer/load_dictionary')
    rospy.wait_for_service('/speech_recognizer/start')
    rospy.wait_for_service('/speech_recognizer/stop')
    rospy.wait_for_service('/speech_synthesizer/synthesize')

    load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    
    load_dict("follow")
    rospy.Subscriber('/speech_recognizer/output', String,speechRecognizerCallback)
    rospy.Subscriber('/speech_synthesizer/status', String,talking_callback)
    
    #servicions y topicos de Kinect
    rospy.wait_for_service('/KinectTracker/SetID')
    
    set_follow_ID = rospy.ServiceProxy('/KinectTracker/SetID', ID)
    rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,estimation_callback)
    rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,init_callback)
    rospy.Subscriber('/KinectTracker_pos_oclusion', OclusionData,oclusion_callback)
    
    debug_print("cargado")
    
    #behavior
    waitingInstruction()
    
    InitFollow()
    
    follow()
    
if __name__ == '__main__':
    main()
