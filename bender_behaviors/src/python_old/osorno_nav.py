#!/usr/bin/env python

# R O S
import roslib; roslib.load_manifest('bender_behaviors')
import rospy

# P y t h o n 
import sys
import math
import time

# Messages
from std_msgs.msg import *
from geometry_msgs.msg import *

# Services
from std_srvs.srv import *

from bender_msgs.msg import *
from bender_srvs.srv import *


class osorno_nav_demo:
    
    def __init__(self):
        
        self.n_name = "osorno_nav_demo"
        rospy.init_node(self.n_name)
        
        self.__service_waiting()
       
        # Clients
        self.client_map_load = rospy.ServiceProxy('/semantic_map_server/load',save_load_Map)
        self.client_map_get = rospy.ServiceProxy('/semantic_map_server/get',sem_map_get)
        self.client_goal_set = rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
        self.client_goal_look = rospy.ServiceProxy('/goalServer/lookToPoseStamped', lookToPoseStamped)
        self.client_speech_synth = rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
        self.client_goal_goPerson = rospy.ServiceProxy('/goalServer/irPersona', lookToPoseStamped)    
        
        # Subscribers
        rospy.Subscriber('/goalServer/feedback', String, self.callback_goalFeedback)
        rospy.Subscriber('/speech_synthesizer/status', String,self.callback_talking)
        #rospy.Subscriber('/WaveUserData',WaveData,self.callback_wave)

        # Publishers
        self.publisher_rosaria = rospy.Publisher('/cmd_vel',Twist)
        self.publisher_kinect_motor = rospy.Publisher('/tilt_angle',Float64)
        
        # Navigation Variables
        self.goal_reached = False
        
        # Speech Variables
        self.speech_talking = False
        
        # Vision Variables
        self.lastWave = WaveData()
        self.lastWaveFlag = False
        
        print "\nReady to work!!\n"

    def __service_waiting(self):

        lista = [
        '/semantic_map_server/load',
        '/semantic_map_server/get',
        '/goalServer/sendGoal',
        '/goalServer/irPersona',
        '/goalServer/lookToPoseStamped',
        '/speech_synthesizer/synthesize'
        ]
        
        for item in lista:
            rospy.loginfo("Waiting for service: " + item)
            rospy.wait_for_service(item)
            
        return

    # N A V I G A T I O N    F U N C T I O N S
    def loadMap(self,mapa):

        try:
            self.client_map_load(mapa)
             
        except rospy.ServiceException, e:
            print "Failed to load map: %s"%e
            
        return
        
    def setGoal(self,destination):
        
        print "voy al " + destination + "!"
        
        destination_obj = self.client_map_get(destination)
        self.__sendGoal(destination_obj.semObj.pose)
        print "Listo: ir a " + destination + "!!"
    
        return
    
    def __sendGoal(self,goal):
        
        self.client_goal_set(goal,0)
        
        count = 0
        count_limit = 30
        
        # Esperar llegada
        self.goal_reached = False
        while not self.goal_reached:
            count += 1
            
            if count > count_limit:
                self.client_goal_set(goal,0)
                count = 0
                
            time.sleep(0.5)
    
        return
    
    def lookToPose(self,ID):
                
        print "lookin to !"+ID
        id_pose = self.client_map_get(ID)
        
        goal = PoseStamped()
        goal.header.frame_id = "/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = id_pose.semObj.pose
        
        self.client_goal_look(goal)
        time.sleep(0.5)
        
        count = 0
        count_limit = 30
        
        # Esperar llegada
        self.goal_reached = False
        while not self.goal_reached:
            count += 1
            
            if count > count_limit:
                self.client_goal_goPerson(goal)
                count = 0
                
            time.sleep(0.5)
    
        return


    def goToPerson(self,x_,y_):
        
        print 'GO TO PERSON'
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "/map"
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose.position.x = x_
        poseStamped.pose.position.y = y_
        poseStamped.pose.position.z = 0.0
        poseStamped.pose.orientation.x = 0.0
        poseStamped.pose.orientation.y = 0.0
        poseStamped.pose.orientation.z = 0.0
        poseStamped.pose.orientation.w = 1.0 
            
        self.client_goal_goPerson(poseStamped)
        time.sleep(0.5)

        
        count = 0
        count_limit = 30
        
        # Esperar llegada
        self.goal_reached = False
        while not self.goal_reached:
            count += 1
            
            if count > count_limit:
                self.client_goal_goPerson(poseStamped)
                count = 0
                
            time.sleep(0.5)
                       
        return


    
    def callback_goalFeedback(self,data):
        
        self.goal_reached = (data.data == "Goal Reached")
        print data
    
    # S P E E C H    F U N C T I O N S
    def talk(self,text):
        
        # para evitar que se quede pegado en un loop infinito
        count=0
        
        print text
        self.client_speech_synth(text)
        self.speech_talking = False
        while not self.speech_talking:
            time.sleep(0.5)
            count = count + 1
            if count > 10:
                break

        while self.speech_talking:
            time.sleep(0.5)
            count = count + 1
            if count > 10:
                break
            
        return

    def callback_talking(self,status):
        
        if status.data == 'Talking':
            self.speech_talking = True
        else:
            self.speech_talking = False
        return
    
#     # V I S I O N    F U N C T I O N S 
#     def callback_wave(self,data):
#        
#         self.lastWave = data
#         print data
#         self.lastWaveFlag = True
#    
#     def waitForWave(timer=10):
#                 
#         self.lastWaveFlag = False
#         count = 0
#         while not self.lastWaveFlag and not count == timer:
#             count+=0.5
#             time.sleep(0.5)
#         if count == timer:
#             return []
#         else:
#             ret = PoseStamped()
#             ret.header.frame_id = '/base_link'
#             ret.header.stamp = rospy.Time.now()
#             ret.pose.position.x = self.lastWave.Z/1000.0
#             ret.pose.position.y = -self.lastWave.X/1000.0
#             ret.pose.position.z = 0
#             
#             return [ret]
#         
#     def tiltKinect(self,angle):
#         
#         phi = Float64()
#         phi.data = angle
#         
#         if angle < -15 or angle > 15:
#             phi.data = 0
#         
#         self.publisher_kinect_motor.publish(phi)

        

if __name__ == '__main__':
    
    obj = osorno_nav_demo()
    obj.loadMap('osorno_demo.sem_map')
   

    print "Working"
    
    print "Saludando"
    obj.talk("Hola soy Bender bla bla")
    obj.talk("En 'esta oportunidad les mostrar'e mis habilidades para recorrer mi nueva habitaci'on")
    obj.talk("Recorrer'e mi nueva casa, para conocerla mejor")

    obj.talk("Donde puedo ir?")   
    obj.setGoal("room_centro")
    obj.talk("debug listo")
    
    obj.setGoal("sala_izq")
    obj.talk("Primero por aqu'i")
    
    obj.setGoal("sala_der")
    obj.talk("O por aqu'i en la sala")
    
    obj.setGoal("room_der")
    obj.talk("Y tambi'en en la otra habitaci'on")
    
    obj.setGoal("room_izq")
    obj.talk("Ya estoy casi listo")
    
    obj.setGoal("centro")
    obj.talk("Ahora buscar'e gente")
    
    listo = False
    poseArray = ["esquina_1","esquina_2","esquina_3","esquina_4","wall_1","wall_2","wall_3","wall_4"]

    lookToPose(poseArray[0])
    lookToPose(poseArray[1])
    lookToPose(poseArray[2])
    lookToPose(poseArray[3])

#     detection = obj.waitForWave(7)
#     
#     if len(detection) != 0:
#         listo = True
# 
#     count = 0
#     while not listo:
#         
#         lookToPose(poseArray[count])
#         detection = obj.waitForWave(7)
#         print detection
#         
#         if len(detection) != 0:
#             listo = True
#         
#         else:
#             obj.talk("Seguir'e buscando")
#             
#         count += 1
#         if count == 4:
#             count=0
        
    obj.talk("Veo a una persona")
    obj.talk("Me acercar'e a saludar")

    obj.talk("Hola, mi nombre es bender")
    
    obj.talk("Uff. Tanto caminar me ha dado sed")
    obj.talk("Ir'e a tomar algo")
    obj.setGoal("mesa_izq")