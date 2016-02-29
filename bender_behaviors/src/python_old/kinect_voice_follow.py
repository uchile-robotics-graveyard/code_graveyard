#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import math
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *

pub = rospy.Publisher('/cmd_vel', Twist)

Pt = -0.0007 # Tralacion
Pr = 0.03   # Rotacion
Pr_occluder = 0#0.0007
safe_dist = 1000
lin_max = 1
thetamax = 1.0#1.0

DIST_OBJ = 1500

follow_ID=-1
kinect_data = -1
follow_R = -1
follow_Theta = -1
lost_ID_distance_threshold = 1000
path_blocked_angle_threshold = 20
max_ID_recovery_angle = 40

# define state Foo
class waitingInstruction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting','instructed'])
        
        self.load_dict = rospy.ServiceProxy('speech_recognizer/load_dictionary', load_dictionary_service)
        self.load_dict("follow")
        self.start = rospy.ServiceProxy('speech_recognizer/start', Empty)
        self.stop = rospy.ServiceProxy('speech_recognizer/stop', Empty)
        
        
        rospy.Subscriber('speech_recognizer/output', String,self.recognition_callback)
        self.talk = rospy.ServiceProxy('speech_synthesizer/synthesize', synthesize)
        
        rospy.Subscriber('speech_synthesizer/status', String,self.talking_callback)
        
        self.instruction_recognized = False
        self.active_state = False
        self.talking = False

    def execute(self,userdata):            
        if self.active_state:
            if self.instruction_recognized:
                if self.talking:
                    return 'waiting'
                else:
                    self.active_state = False
                    self.instruction_recognized = False
                    self.stop()
                    self.talk("I will follow you. Wait for me to recognize you.")
                    time.sleep(1.3)
                    return 'instructed'
            else:
                return 'waiting'
        else:
            
            self.start()
            self.active_state = False#True
            self.instruction_recognized = False
            return 'instructed'#'waiting'
    
    def recognition_callback(self, recognition):
        if self.active_state and self.instruction_recognized == False:
            if recognition.data == "bender follow me":
                self.instruction_recognized = True
            else:
                if self.talking==False:
                    self.talk("I did not understand. Could you repeat that?")
                    time.sleep(1.3)
                self.instruction_recognized = False
                
    def talking_callback(self, status):
        if status.data == "Talking":
            self.talking = True
        else:
            self.talking = False

class InitFollow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Is Init','Is Not Init'])
        self.counter = 0
        self.is_init=False
        rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,self.callback)
        self.talk = rospy.ServiceProxy('speech_synthesizer/synthesize', synthesize)
        rospy.Subscriber('speech_synthesizer/status', String,self.talking_callback)
        self.set_follow_ID = rospy.ServiceProxy('KinectTracker/SetID', ID)
        
        self.active_state = False
        self.talking = False

    def execute(self, userdata):
        if self.active_state:
            if self.is_init:
                if self.talking:
                    return 'Is Not Init'
                else:
                    self.talk("Ok, I will start following you now.")
                    time.sleep(1.3)
                    return 'Is Init'
            else:
                if self.talking:
                    time.sleep(6)
                else:
                    self.talk("Please stand one meter in front of me.")
                    time.sleep(1.3)
                return 'Is Not Init'
        else:
            if self.talking==False:
                self.talk("Please stand one meter in front of me.")
            self.active_state = True
            self.is_init=False
            return 'Is Not Init'
            
    def talking_callback(self, status):
            if status.data == "Talking":
                self.talking = True
            else:
                self.talking = False
    
    def callback(self, data):
        global follow_ID,follow_R,follow_Theta
        if self.is_init:
            return
        for i in range(len(data.ID)):
            if abs(data.Distance[i]-DIST_OBJ) < 300 and abs(data.Theta[i]) < 5 :
                follow_ID=data.ID[i]
                follow_R=DIST_OBJ
                follow_Theta=0
                self.set_follow_ID(follow_ID)
                self.is_init=True
                
                
            
        


# define state Bar
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Following'])
        self.counter=0
        self.twist = Twist() # inicializa en ceros
        self.lost=True
        self.oclusion_ID=-3
        self.oclusion_R=0
        self.oclusion_Theta=0
        self.oclusion_Position=0
        self.oclusion_Orientation=0
        self.bloking_ind=-3
        self.active_state=False
        self.path_blocked=False
        self.occluders_in_range=False
        self.path_bloqued_update = 0
        self.free_path_update = 1
        self.lost_update = 0.8
        rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,self.estimation_callback)
        rospy.Subscriber('/KinectTracker_pos_oclusion', OclusionData,self.oclusion_callback)
        self.set_follow_ID = rospy.ServiceProxy('KinectTracker/SetID', ID)
        self.file = open("/home/bendervision/fuerte_workspace/bender_behaviors/debug.log","w")
  
    def execute(self, userdata):
        #time.sleep(2)
        global follow_ID,follow_R,follow_Theta
        if self.active_state:
            if self.lost:
                if follow_Theta > max_ID_recovery_angle or follow_Theta < -max_ID_recovery_angle:
                    self.twist.angular.z=self.twist.angular.z*(1-self.lost_update)
                    self.twist.linear.x=self.twist.linear.x*(1-self.lost_update)
                else:
                    self.twist.angular.z=0
                    self.twist.linear.x=0
                pub.publish(self.twist) 
                return 'Following'
    
            if self.path_blocked:
                self.twist.angular.z = 0
                self.twist.linear.x = 0
                pub.publish(self.twist)
                
                time.sleep(0.5)
                
                return 'Folowing'
#                 occluder_x = self.oclusion_R[self.bloking_ind]*math.cos(self.oclusion_Theta[self.bloking_ind]*math.pi/180)
#                 occluder_y = self.oclusion_R[self.bloking_ind]*math.sin(self.oclusion_Theta[self.bloking_ind]*math.pi/180)
#                 self.file.write("oclusion position:"+str(self.oclusion_Position[self.bloking_ind])+"\n")
#                 self.file.write("occluder y:"+str(occluder_y)+"\n")
#                 if self.oclusion_Position[self.bloking_ind] == "LEFT":
#                     self.twist.angular.z =(Pr*follow_Theta+Pr_occluder*min(0,occluder_y-safe_dist))*self.path_bloqued_update+self.twist.angular.z*(1-self.path_bloqued_update)
#                     self.twist.angular.z=min(self.twist.angular.z,thetamax)
#                     self.twist.angular.z=max(self.twist.angular.z,-thetamax)
#                     
#                     self.twist.linear.x = Pt*(DIST_OBJ-follow_R)
#                     self.twist.linear.x = min(self.twist.linear.x,lin_max)
#                     self.twist.linear.x = max(self.twist.linear.x,-lin_max)
#                 elif self.oclusion_Position[self.bloking_ind] == "RIGHT":
#                     self.twist.angular.z =Pr*follow_Theta+Pr_occluder*max(0,occluder_y+safe_dist)*self.path_bloqued_update+self.twist.angular.z*(1-self.path_bloqued_update)
#                     self.twist.angular.z=min(self.twist.angular.z,thetamax)
#                     self.twist.angular.z=max(self.twist.angular.z,-thetamax)
#                     
#                     self.twist.linear.x = Pt*(DIST_OBJ-follow_R)
#                     self.twist.linear.x = min(self.twist.linear.x,lin_max)
#                     self.twist.linear.x = max(self.twist.linear.x,-lin_max)
#                 else:                
#                     self.twist.angular.z = self.twist.angular.z*(1-self.path_bloqued_update)
#                     self.twist.linear.x = self.twist.linear.x*(1-self.path_bloqued_update)
#                     self.file.write("Nothing to do here"+"\n")
<<<<<<< HEAD
                self.file.write("blocking ind: "+str(self.blocking_ind)+"\n")
                if self.oclusion_Position[self.blocking_ind] == "LEFT":
                    self.file.write("occlusion left, starting evasion"+"\n")
                    
                    self.twist.angular.z = -0.5
                    self.twist.linear.x = 0.5
                    pub.publish(self.twist)
                    time.sleep(1)
                    self.twist.angular.z = 0
                    self.twist.linear.x =  1
                    pub.publish(self.twist)
                    time.sleep(1.5)
                    self.twist.angular.z = 0.5
                    self.twist.linear.x =  0.5
                    pub.publish(self.twist)
                    time.sleep(2)
                    self.twist.angular.z = 0
                    self.twist.linear.x =  1
                    pub.publish(self.twist)
                    time.sleep(1.5)
                    self.twist.angular.z = -0.5
                    self.twist.linear.x = 0.5
                    pub.publish(self.twist)
                    time.sleep(1)
                    
                    return 'Following'
                elif self.oclusion_Position[self.blocking_ind] == "RIGHT":
                    self.file.write("occlusion right, starting evasion"+"\n")
                    
                    self.twist.angular.z = 0.5
                    self.twist.linear.x = 0.5
                    pub.publish(self.twist)
                    time.sleep(1)
                    self.twist.angular.z = 0
                    self.twist.linear.x =  1
                    pub.publish(self.twist)
                    time.sleep(1.5)
                    self.twist.angular.z = -0.5
                    self.twist.linear.x =  0.5
                    pub.publish(self.twist)
                    time.sleep(2)
                    self.twist.angular.z = 0
                    self.twist.linear.x =  1
                    pub.publish(self.twist)
                    time.sleep(1.5)
                    self.twist.angular.z = 0.5
                    self.twist.linear.x = 0.5
                    pub.publish(self.twist)
                    time.sleep(1)
                    return 'Following'
                else:
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0
                    pub.publish(self.twist)
                    return 'Following'
=======



#                 self.file.write("blocking ind: "+str(self.blocking_ind)+"\n")
#                 if self.oclusion_Position[self.blocking_ind] == "LEFT":
#                     self.file.write("occlusion left, starting evasion"+"\n")
#                     
#                     self.twist.angular.z = -0.7
#                     self.twist.linear.x = 0.5
#                     pub.publish(self.twist)
#                     time.sleep(1.5)
#                     self.twist.angular.z = 0
#                     self.twist.linear.x =  1
#                     pub.publish(self.twist)
#                     time.sleep(1.5)
#                     self.twist.angular.z = 0.7
#                     self.twist.linear.x =  0.25
#                     pub.publish(self.twist)
#                     time.sleep(3)
#                     self.twist.angular.z = 0
#                     self.twist.linear.x =  1
#                     pub.publish(self.twist)
#                     time.sleep(1.5)
#                     self.twist.angular.z = 0.7
#                     self.twist.linear.x = 0.5
#                     pub.publish(self.twist)
#                     time.sleep(1.5)
#                     
#                     return 'Following'
#                 elif self.oclusion_Position[self.blocking_ind] == "RIGHT":
#                     self.file.write("occlusion right, starting evasion"+"\n")
#                     
#                     self.twist.angular.z = 0.7
#                     self.twist.linear.x = 0.5
#                     pub.publish(self.twist)
#                     time.sleep(1)
#                     self.twist.angular.z = 0
#                     self.twist.linear.x =  1
#                     pub.publish(self.twist)
#                     time.sleep(1)
#                     self.twist.angular.z = -0.7
#                     self.twist.linear.x =  0.25
#                     pub.publish(self.twist)
#                     time.sleep(2)
#                     self.twist.angular.z = 0
#                     self.twist.linear.x =  1
#                     pub.publish(self.twist)
#                     time.sleep(1)
#                     self.twist.angular.z = -0.7
#                     self.twist.linear.x = 0.5
#                     pub.publish(self.twist)
#                     time.sleep(1)
#                     return 'Following'
#                 else:
#                     self.twist.angular.z = 0
#                     self.twist.linear.x = 0
#                     pub.publish(self.twist)
#                     return 'Following'
>>>>>>> 8530459a56a22ce7db11d215b9b377e6f8e4b086
                    
            else:                
                self.twist.angular.z =Pr*follow_Theta*self.free_path_update+self.twist.angular.z*(1-self.free_path_update)
                self.twist.angular.z=min(self.twist.angular.z,thetamax)
                self.twist.angular.z=max(self.twist.angular.z,-thetamax)
                
                self.twist.linear.x = Pt*(DIST_OBJ-follow_R)
                self.twist.linear.x = min(self.twist.linear.x,lin_max)
                self.twist.linear.x = max(self.twist.linear.x,-lin_max)
            
            #self.file.write("move : "+str(self.twist.linear.x)+"\n")
            #self.file.write("rotate : "+str(self.twist.angular.z)+"\n")
                
            pub.publish(self.twist)
        else:
            self.path_blocked = False
            self.active_state=True
            
            self.twist.angular.z =Pr*follow_Theta
            self.twist.angular.z=min(self.twist.angular.z,thetamax)
            self.twist.angular.z=max(self.twist.angular.z,-thetamax)
            
            self.twist.linear.x = Pt*(DIST_OBJ-follow_R)
            self.twist.linear.x = min(self.twist.linear.x,lin_max)
            self.twist.linear.x = max(self.twist.linear.x,-lin_max)
            
        return 'Following'
        
    def estimation_callback(self, data):
        global follow_ID,follow_R,follow_Theta,kinect_data
        index=-1
        for i in range(len(data.ID)):
            if data.ID[i] == follow_ID:
                index=i
                break
        if index == -1:
            if self.path_blocked:
                self.lost=True
                self.file.write("lost in -1 y path blocked"+"\n")
            else:
                if follow_Theta > max_ID_recovery_angle or follow_Theta < -max_ID_recovery_angle:
                    self.lost=True
                    self.file.write("lost ind -1 y fuera de angle recovery range"+"\n")
                else:
                    min_dist=9999999999999999
                    min_ind=-1
                    for i in range(len(data.ID)):
                        dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[i]*math.cos(data.Theta[i]*math.pi/180)
                        dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[i]*math.sin(data.Theta[i]*math.pi/180)
                        dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                        if dist < min_dist:
                            min_dist = dist
                            self.file.write("min dist: "+str(min_dist)+"\n")
                            min_ind = i
                            
                    if min_dist < lost_ID_distance_threshold:
                        follow_ID=data.ID[min_ind]
                        follow_R=data.Distance[min_ind]
                        follow_Theta=data.Theta[min_ind]
                        self.file.write("Kinect ind=-1 set follow ID a: "+str(follow_ID)+"\n")
                        self.set_follow_ID(follow_ID)
                        self.lost=False
        else:
            if self.path_blocked:
                self.lost=False
                follow_R=data.Distance[index]
                follow_Theta=data.Theta[index]
            else:
                dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[index]*math.cos(data.Theta[index]*math.pi/180)
                dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[index]*math.sin(data.Theta[index]*math.pi/180)
                dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                if dist < lost_ID_distance_threshold:
                    self.lost=False
                    follow_R=data.Distance[index]
                    follow_Theta=data.Theta[index]
                else:
                    for i in range(len(data.ID)):
                        dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-data.Distance[i]*math.cos(data.Theta[i]*math.pi/180)
                        dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-data.Distance[i]*math.sin(data.Theta[i]*math.pi/180)
                        dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                        if dist < min_dist:
                            min_dist = dist
                            self.file.write("min dist: "+str(min_dist)+"\n")
                            min_ind = i
                        
                    if min_dist < lost_ID_distance_threshold:
                        follow_ID=data.ID[min_ind]
                        follow_R=data.Distance[min_ind]
                        follow_Theta=data.Theta[min_ind]
                        self.file.write("Kinect =ID pero lejos set follow ID a: "+str(follow_ID)+"\n")
                        self.set_follow_ID(follow_ID)
                        self.lost=False
                        
        kinect_data=data
        
        
    def oclusion_callback(self,data):
        global follow_ID,follow_R,follow_Theta
        if self.active_state:
            kinect_data_temp = kinect_data
            self.oclusion_ID = data.ID
            self.oclusion_R = data.dist
            self.oclusion_Theta = data.theta
            self.oclusion_Position = data.position
            self.oclusion_Direction = data.direction
    
            if self.lost:
                min_dist=9999999999999999
                min_ind=-1
                for i in range(len(kinect_data_temp.ID)):
                    occluder = False
                    for j in range(len(self.oclusion_ID)):
                        occluder = occluder or self.oclusion_ID[j]==kinect_data_temp.ID[i]
                    if occluder==False:
                        self.file.write("persona no es oclusora: "+str(kinect_data_temp.ID[i])+"\n")
                        dist_x = follow_R*math.cos(follow_Theta*math.pi/180)-kinect_data_temp.Distance[i]*math.cos(kinect_data_temp.Theta[i]*math.pi/180)
                        dist_y = follow_R*math.sin(follow_Theta*math.pi/180)-kinect_data_temp.Distance[i]*math.sin(kinect_data_temp.Theta[i]*math.pi/180)
                        dist = math.pow(math.pow(dist_x,2)+math.pow(dist_y,2), 0.5)
                        if dist < min_dist:
                            min_dist = dist
                            self.file.write("min dist: "+str(min_dist)+"\n")
                            min_ind = i
    
                if min_dist < lost_ID_distance_threshold and kinect_data_temp.ID[min_ind]!=follow_ID:
                    follow_ID=kinect_data_temp.ID[min_ind]
                    follow_R=kinect_data_temp.Distance[min_ind]
                    follow_Theta=kinect_data_temp.Theta[min_ind]
                    self.file.write("Occlusion set follow ID a: "+str(follow_ID)+"\n")
                    self.set_follow_ID(follow_ID)
    
            for i in range(len(self.oclusion_ID)):
                if abs(self.oclusion_Theta[i]-follow_Theta)<path_blocked_angle_threshold:
                    self.path_blocked = True
                    self.file.write("path bloqued by index: "+str(elf.bloking_ind)+"\n")
                    self.bloking_ind = i
                else: 
                    self.path_blocked = False
                    self.blocking_ind = -3
                    
            if len(self.oclusion_ID)==0:
                self.path_blocked = False
                self.blocking_ind = -3
                

def main():
    rospy.init_node('follow')
    
#     time.sleep(5)
#     twist = Twist()
#     
#     twist.angular.z = 0
#     twist.linear.x = 0.5
#     pub.publish(twist)
#     time.sleep(3)
#     
#     twist.angular.z = -0.5
#     twist.linear.x = 0.5
#     pub.publish(twist)
#     time.sleep(1)
#     twist.angular.z = 0
#     twist.linear.x =  1
#     pub.publish(twist)
#     time.sleep(1.5)
#     twist.angular.z = 0.7
#     twist.linear.x =  0.5
#     pub.publish(twist)
#     time.sleep(2)
#     twist.angular.z = 0
#     twist.linear.x =  1
#     pub.publish(twist)
#     time.sleep(1.5)
#     twist.angular.z = -0.5
#     twist.linear.x = 0.5
#     pub.publish(twist)
#     time.sleep(1)
#               
#     twist.angular.z = 0
#     twist.linear.x = 0.5
#     pub.publish(twist)
#     time.sleep(1)
#     
#     twist.angular.z = 0
#     twist.linear.x = 0
#     pub.publish(twist)
#     return
    #rospy.wait_for_server('/camera_service')
    #print "waiting for camera_service"
    #camera_service = rospy.ServiceProxy('camera_service', ImageService)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('waitingInstruction',waitingInstruction(),
                               transitions={'waiting':'waitingInstruction',
                                            'instructed':'InitFollow'})
        
        smach.StateMachine.add('InitFollow', InitFollow(), 
                               transitions={'Is Init':'Follow', 
                                            'Is Not Init':'InitFollow'})
        smach.StateMachine.add('Follow', Follow(), 
                               transitions={'Following':'Follow'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
