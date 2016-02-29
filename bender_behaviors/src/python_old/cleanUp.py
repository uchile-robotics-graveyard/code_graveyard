#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import math
import time

from geometry_msgs.msg import *
from std_srvs.srv import *
from rospy.timer import sleep
from bender_msgs.msg import *
from bender_srvs.srv import *

class cleanUp:
    
    def __init__(self):
        
        rospy.init_node('cleanUp')

        # Variables
        self.goal_reached = False
        self.objeto_encontrado = False
        self.otro_objeto_encontrado = False
        self.object_category = ""
        self.object_name = ""
        self.obj_point = Point()
        self.otro_obj_point = Point()

        # Servicios        
        self.sem_get_srv = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
        self.sem_load_srv = rospy.ServiceProxy('/semantic_map_server/load', save_load_Map)
        self.setGoal_srv = rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
        self.cancelGoal_srv = rospy.ServiceProxy('/goalServer/cancelGoal', Empty)
        self.doorDetection_srv = rospy.ServiceProxy('/door_open_detector/isopen', DoorDetector)
        self.speech_synthesize_srv = rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
        self.planeon = rospy.ServiceProxy('/planeOn', planeOn)
        self.sifton = rospy.ServiceProxy('/siftOn', siftOn)
        self.detect = rospy.ServiceProxy('/arm_vision_interface/detect_obj', ObjectDetection)
        self.pre1 = rospy.ServiceProxy('/right_arm/posicion_premanipulacion1', Dummy)
        self.pre2 = rospy.ServiceProxy('/right_arm/posicion_premanipulacion2', Dummy)
        self.abrir = rospy.ServiceProxy('/right_arm/abrir_grip', Dummy)           
        self.plan1 = rospy.ServiceProxy('/right_arm/grasp', PlanningGoalCartesian)
        self.orient_grip = rospy.ServiceProxy('/right_arm/orientar_grip', Dummy)
        self.cerrar = rospy.ServiceProxy('/right_arm/cerrar_grip', LoadMode)
        self.post1 = rospy.ServiceProxy('/right_arm/posicion_postmanipulacion1', Dummy)
        self.inicial = rospy.ServiceProxy('/right_arm/posicion_inicial', Dummy)
        self.planeoff = rospy.ServiceProxy('/planeOff', planeOff)
        self.siftoff = rospy.ServiceProxy('/siftOff', siftOff)
        self.detect = rospy.ServiceProxy('/arm_vision_interface/reset_obj', Dummy)

        self.srv_waiting()
 
        # Suscribers
        self.goalFB = rospy.Subscriber('/goalServer/feedback', String, self.goalFBcallback)
        
        # Publishers
        self.initialPosePub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped)
        self.rosaria_pub = rospy.Publisher('/cmd_vel', Twist)
        
        
        # Cargar mapa
        self.sem_load_srv("eindhoven.sem_map")
                
        print "\nReady to work :D!!\n"

    def srv_waiting(self):
        self.waitSrv('/semantic_map_server/get')
        self.waitSrv('/semantic_map_server/load')
        self.waitSrv('/goalServer/sendGoal')
        self.waitSrv('/door_open_detector/isopen')
        self.waitSrv('/speech_synthesizer/synthesize')
        
        # Brazo
        self.waitSrv('/planeOn')
        self.waitSrv('/siftOn')
        self.waitSrv('/arm_vision_interface/detect_obj')
        self.waitSrv('/right_arm/posicion_premanipulacion1')
        self.waitSrv('/right_arm/posicion_premanipulacion2')
        self.waitSrv('/right_arm/abrir_grip')
        self.waitSrv('/right_arm/grasp')
        self.waitSrv('/right_arm/orientar_grip')
        self.waitSrv('/right_arm/cerrar_grip')
        self.waitSrv('/right_arm/posicion_postmanipulacion1')
        self.waitSrv('/right_arm/posicion_inicial')
        self.waitSrv('/planeOff')
        self.waitSrv('/siftOff')
        
    def waitSrv(self,srv):
    
        print "Esperando servicio " + srv 
        rospy.wait_for_service(srv)
        print "OK"
            
    def waitForOpenDoor(self,distance=100):
        
        print "esperando puerta abierta"
        while not self.doorDetection_srv.call(distance).DoorState:
            time.sleep(1)

    def setInitialPose(self,pose="start"):
            
        # FALTA UN  .call(%) ????
        resp = self.sem_get_srv(pose)
        
        initialPose = PoseWithCovarianceStamped()
        initialPose.header.frame_id = "/map"
        initialPose.header.stamp = rospy.Time.now()
        initialPose.pose.pose = resp.semObj.pose
        initialPose.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        self.initialPosePub.publish(initialPose)

    def setGoal(self,destination):
        
        print "al " + destination + "!"
        goal = self.sem_get_srv(destination)
        self.setGoal_srv(goal.semObj.pose,0)
        self.goal_reached = False
        
        time.sleep(0.5)
            
        return

    def setGoalPose(self,pose):
        
        print "al " + destination + "!"
        self.setGoal_srv(pose,0)
        
        time.sleep(0.5)
        
        # Esperar
        self.goal_reached = False
        while not self.goal_reached:
            if self.objeto_encontrado == True:
                self.cancelGoal()
                # Ir a dejar el objeto a su posicion
                break
            time.sleep(0.5)
            
        return

    def cancelGoal(self):
        self.cancelGoal_srv()

    def arenaEntrance(self,t_ini=5,duration=5):
        
        print "esperando un poco"
        time.sleep(t_ini)
        
        #avanzar
        print "avanzando"
        order = Twist()
        order.linear.x=0.3
        order.angular.z=0.0
        self.rosaria_pub.publish(order)
        
        #esperar
        print "esperando otro poco"
        time.sleep(duration)
        
        #frenar
        print "frenando"
        order.linear.x=0.0
        order.angular.z=0.0
        self.rosaria_pub.publish(order)
        
        return

    def talk(self,text="",time=0.1):
        print text
        self.speech_synthesize_srv(text)
        time.sleep(time)
    
    # .......... Callbacks .....................................
    #-----------------------------------------------------------
 
    def goalFBcallback(self,data):
        
        self.goal_reached = (data.data == "Goal Reached")
        print data
        

    def seekObject(self):
        
        self.sifton(0)
        time.sleep(3)
        
        self.objeto_encontrado = False        
        
        detected_obj = self.detect()
        for i in range(len(detected_obj.name)):
            if len(detected_obj.name[i]) > 1:
                
                self.objeto_encontrado = True
                self.object_name = detected_obj.name[i] 
                self.object_category = detected_obj.type[i]
                self.obj_point.x = detected_obj.x[i]
                self.obj_point.y = detected_obj.y[i]
                self.obj_point.z = detected_obj.z[i]        
                break
            
        self.siftoff(0)
            
    def seekAnotherObject(self):
        
        self.sifton(0)
        time.sleep(3)
        self.otro_objeto_encontrado = False 
        
        detected_obj = self.detect()
        for i in range(len(detected_obj.name)):
            if len(detected_obj.name[i]) > 1:
                
                self.otro_objeto_encontrado = True
                self.otro_obj_point.x = detected_obj.x[i]
                self.otro_obj_point.y = detected_obj.y[i]
                self.otro_obj_point.z = detected_obj.z[i]        
                break

    def depositarObjeto(self):
        
        # depositar stuff
        self.talk("I dont know how to put this away")
        self.talk("Please take it away from my hand")
        #self.inicial()
        self.planeoff(0)
        return
    
    def tomarObjeto(self):
        # depositar stuff
        self.planeon(0)
        self.talk("Tomando objeto")
        self.pre1()
        self.pre2()
        self.abrir()
        self.plan1(x,y,z)
        self.orient_grip()
        self.cerrar(1)
        self.post1()
        
        return

# ----------------------- BEHAVIOR ----------------------------

if __name__ == '__main__':
    
    node = cleanUp()   






    # Recorrer buscando objetos en estos hacia estos lugares
    lugares = ["p0", "p1","p2","p3"]
    place_index = 0
    STATE = "INIT"
    GRIP_EMPTY = True
    obj_info = SemanticObject()
    equivar_info = SemanticObject()
    move_time = 2 # [segundos]
    
    while True:
        
        if STATE == "INIT":
            #DONE
            
            # Pose inicial
            node.talk("Setting initial pose")
            node.setInitialPose("start")
            
            # Esperar puerta abierta
            node.talk("waiting for open door")
            node.waitForOpenDoor()
         
            # esperar un poco
            node.talk("Entering the arena")
            node.arenaEntrance(2,5) 
            time.sleep(1)
            
            STATE = "setGoal"
         
        elif STATE == "setGoal":
            #DONE
            node.setGoal(lugares[place_index])
            
            node.objeto_encontrado = False
            
            while self.goal_reached == False:
                time.sleep(1)
            
            self.goal_reached = True
            
            node.objeto_encontrado = False            
            node.seekObject()

            if node.objeto_encontrado:
                STATE = "manipular"
            else:
                place_index+=1
                STATE = "setGoal"
                
                if place_index>=len(lugares):
                    STATE="END"
                    
        elif STATE == "manipular":
            
            node.talk("I found a " + node.object_name)
            node.tomarObjeto()
            obj_info = node.sem_get_srv(node.object_category)
            node.talk("it is a " + obj_info.semObj.id + " so i am going to the " + obj_info.semObj.type)
            node.setGoalPose(obj_info.semObj.pose)
            
            self.goal_reached = False
            while self.goal_reached == False:
                time.sleep(1)
                
            node.depositarObjeto()
            place_index+=1
            
            STATE = "setGoal"
            
            if place_index>=len(lugares):
                STATE="END"
                
        elif STATE == "END":
            break

    
    node.setGoal("exit ini")
    node.talk("hasta la vista. baby")
