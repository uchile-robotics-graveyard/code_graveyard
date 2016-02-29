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

twist = Twist()
follow_ID=-1
follow_R = -1
follow_Theta = -1
kinect_data = -1

path_blocked=False
lost=True

oclusion_ID = -1
oclusion_R = -1
oclusion_Theta = 0
oclusion_Position = -1
oclusion_Direction = -1
blocking_ind = -1

set_follow_ID = -1

pub = ""
twist = -1

#parametros globales
Pt = -0.0007 # Tralacion
Pr = 0.03   # Rotacion
follow_distace = 1500
lin_max = 0
thetamax = 1#1.0
path_blocked_angle_threshold = 10
max_ID_recovery_angle = 40
lost_ID_distance_threshold=2000
histeresis_threshold_linear=0.1;
histeresis_threshold_theta=0.1;
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
    if speech_talking:
        return
    print data.data
    if data.data==instruction_to_recognize:
        instruction_recognized=True
    else:
        speech_bad_detection=True
    return;

def estimation_callback(data):
    global follow_ID,follow_R,follow_Theta,kinect_data,is_init
    if not is_init:
        return
    index=-1
    for i in range(len(data.ID)):
        if data.ID[i] == follow_ID:
            index=i
            break
    if index == -1:
        if path_blocked:
            lost=True
            debug_print("estimation_callback: index==-1,path_bloqued==True -> lost=True")
        else:
            if follow_Theta > max_ID_recovery_angle or follow_Theta < -max_ID_recovery_angle:
                lost=True
                debug_print("estimation_callback: index==-1,path_bloqued==False,follow_theta outside max_ID_recovery_angle -> lost=True")
            else:
                min_dist=9999999999999999
                min_ind=-1
                for i in range(len(data.ID)):
                    dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[i]*math.cos(data.Theta[i]*math.pi/180)
                    dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[i]*math.sin(data.Theta[i]*math.pi/180)
                    dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                    if dist < min_dist:
                        min_dist = dist
                        min_ind = i
                        
                if min_dist < lost_ID_distance_threshold:
                    follow_ID=data.ID[min_ind]
                    follow_R=data.Distance[min_ind]
                    follow_Theta=data.Theta[min_ind]
                    debug_print("estimation_callback: index==-1,path_bloqued==False,recovery atemt succes --> follow ID: "+str(follow_ID))
                    set_follow_ID(follow_ID)
                    lost=False
                else:
                    if len(data.ID)==1:
                        follow_ID=data.ID[min_ind]
                        follow_R=data.Distance[min_ind]
                        follow_Theta=data.Theta[min_ind]
                        debug_print("estimation_callback: index==-1,path_bloqued==False,recovery atemt succes, only 1 person in frame --> follow ID: "+str(follow_ID))
                        set_follow_ID(follow_ID)
                        lost=False
                    else:
                        lost=True
                        debug_print("estimation_callback: index==-1,path_bloqued==False,recovery atemt failure --> min_dist: "+str(min_dist))
    else:
        if abs(data.Distance[index])>lost_ID_distance_threshold:
            if path_blocked:
                lost=True
                debug_print("estimation_callback: ID found,distance to big ID recovery,path_bloqued==True -> lost=True")
            else:
                dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[index]*math.cos(data.Theta[index]*math.pi/180)
                dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[index]*math.sin(data.Theta[index]*math.pi/180)
                dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                
                min_ind=-1
                for i in range(len(data.ID)):
                    dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[i]*math.cos(data.Theta[i]*math.pi/180)
                    dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[i]*math.sin(data.Theta[i]*math.pi/180)
                    dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                    if dist < min_dist:
                        min_dist = dist
                        min_ind = i
                if min_ind==-1:
                    follow_R=data.Distance[index]
                    follow_Theta=data.Theta[index]
                    lost=False
                else:
                    follow_ID=data.ID[min_ind]
                    follow_R=data.Distance[min_ind]
                    follow_Theta=data.Theta[min_ind]
                    debug_print("estimation_callback: ID found,distance to big ID recovery,changing to closest ID: "+str(follow_ID))
                    set_follow_ID(follow_ID)
                    lost=False
                return
        else:
            follow_R=data.Distance[index]
            follow_Theta=data.Theta[index]
            lost=False
    kinect_data=data

def oclusion_callback(data):
    global follow_ID,follow_R,follow_Theta,oclusion_ID,oclusion_R,oclusion_Theta,oclusion_Position,oclusion_Direction,blocking_ind,lost,is_init
    if not is_init:
        return
    kinect_data_temp = kinect_data
    oclusion_ID = data.ID
    oclusion_R = data.dist
    oclusion_Theta = data.theta
    oclusion_Position = data.position
    oclusion_Direction = data.direction

    if lost:
        min_dist=9999999999999999
        min_ind=-1
        for i in range(len(kinect_data_temp.ID)):
            occluder = False
            for j in range(len(oclusion_ID)):
                occluder = occluder or oclusion_ID[j]==kinect_data_temp.ID[i]
            if occluder==False:
                dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-kinect_data_temp.Distance[i]*math.cos(kinect_data_temp.Theta[i]*math.pi/180)
                dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-kinect_data_temp.Distance[i]*math.sin(kinect_data_temp.Theta[i]*math.pi/180)
                dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                if dist < min_dist:
                    min_dist = dist
                    min_ind = i

        if min_dist < lost_ID_distance_threshold and kinect_data_temp.ID[min_ind]!=follow_ID:
            follow_ID=kinect_data_temp.ID[min_ind]
            follow_R=kinect_data_temp.Distance[min_ind]
            follow_Theta=kinect_data_temp.Theta[min_ind]
            debug_print("oclusion_callback: lost==True --> changin ID, min dist ID: "+str(follow_ID))
            set_follow_ID(follow_ID)
            lost=False
            

    for i in range(len(oclusion_ID)):
        if abs(oclusion_Theta[i]-follow_Theta)<path_blocked_angle_threshold:
            path_blocked = True
            bloking_ind = i
            debug_print("oclusion_callback: path bloqued by index: "+str(bloking_ind))
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
        if abs(data.Distance[i]-follow_distace) < 300 and abs(data.Theta[i]) < 5 :
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
    
    while not instruction_recognized:
        if speech_bad_detection:
            talk("I did not understand. Would you please repeat?")
            speech_bad_detection=False
        time.sleep(1)
    
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
        time.sleep(6)
    talk("Ok, I will start following you now.")

def follow():
    global follow_ID,follow_R,follow_Theta,oclusion_ID,oclusion_R,oclusion_Theta,oclusion_Position,oclusion_Direction,blocking_ind,lost,twist
     
    counter=0;
    following=True
    while following:
        if path_bloqued:
            if counter==0:
                talk("please do not block my path")
            twist.angular.z=0
            twist.linear.x=0
            pub.publish(twist)
            time.sleep(0.5)
            counter=conuter+1
        elif lost:
            #chekear angulo
            twist.angular.z=0
            twist.linear.x=1
            twist.linear.x = min(twist.linear.x,lin_max)
            twist.linear.x = max(twist.linear.x,-lin_max)
            pub.publish(twist)
        else:
            counter=0
            twist.linear.x = Pt*(follow_distace-follow_R)
            twist.linear.x = min(twist.linear.x,lin_max)
            twist.linear.x = max(twist.linear.x,-lin_max)
             
            twist.angular.z = Pr*follow_Theta
            twist.angular.z=min(twist.angular.z,thetamax)
            twist.angular.z=max(twist.angular.z,-thetamax)
             
            if twist.linear.x<histeresis_threshold_linear:
                twist.linear.x =0
             
            if twist.angular.z<histeresis_threshold_theta:
                twist.angular.z = 0
             
            pub.publish(twist)
    return

def listen():
    speechRecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    
    global instruction_recognized
    
    speechRecoStart()
    debug_print("Reconocimiento activado, escuchando...")
    
    return

def main():
    global set_follow_ID,pub,twist
    
    rospy.init_node('follow')
    
    #servicions y topicos de speech
    rospy.wait_for_service('/speech_recognizer/load_dictionary')
    #rospy.wait_for_service('/speech_recognizer/start')
    #rospy.wait_for_service('/speech_recognizer/stop')
    #rospy.wait_for_service('/speech_synthesizer/synthesize')

    load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    
    load_dict("test_cocktail")
    rospy.Subscriber('/speech_recognizer/output', String,speechRecognizerCallback)
    rospy.Subscriber('/speech_synthesizer/status', String,talking_callback)
    
    #servicions y topicos de Kinect
    #rospy.wait_for_service('/KinectTracker/SetID')
    
    set_follow_ID = rospy.ServiceProxy('/KinectTracker/SetID', ID)
    rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,estimation_callback)
    rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,init_callback)
    rospy.Subscriber('/KinectTracker_pos_oclusion', OclusionData,oclusion_callback)
    
    
    #servicions y topicos base
    pub = rospy.Publisher('/cmd_vel', Twist)
    twist = Twist()
    
    #behavior
    listen()
#     waitingInstruction()
    
#     InitFollow()

#     follow()
    
if __name__ == '__main__':
    main()
