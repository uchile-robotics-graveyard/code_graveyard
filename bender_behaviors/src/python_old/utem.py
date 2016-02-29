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

class utem_demo:
    
    def __init__(self):
        
        self.n_name = "utem_demo"
        
        rospy.init_node(self.n_name)
        
        self.__service_waiting()
       
        # Clients
        self.client_map_load = rospy.ServiceProxy('/semantic_map_server/load',save_load_Map)
        self.client_map_get = rospy.ServiceProxy('/semantic_map_server/get',sem_map_get)
        self.client_goal_set = rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
        self.client_goal_approach = rospy.ServiceProxy('/goalServer/approach_to_wall',approachToWall)
        self.client_speech_synth = rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
        self.client_speech_RecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
        self.client_speech_RecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
        self.client_speech_load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)

        # Subscribers
        rospy.Subscriber('/goalServer/feedback', String, self.callback_goalFeedback)
        rospy.Subscriber('/speech_synthesizer/status', String,self.callback_talking)
        rospy.Subscriber('/speech_recognizer/output', String, self.callback_speechRecognizer)

        # Publishers
        self.publisher_rosaria = rospy.Publisher('/cmd_vel',Twist)
        

        # Navigation Variables
        self.goal_reached = False
        
        # Speech Variables
        self.confirmation = ['yes' , 'no']
        self.names = ['michael' , 'jessica' , 'christopher' , 'ashley' , 'mathew' , 'brittany' , 'joshua' , 'amanda' , 'daniel' , 'samantha' , 'david' , 'sarah' , 'andrew' , 'stephanie' , 'james' , 'jennifer' , 'justin' , 'elizabeth' , 'joseph' , 'lauren']
        self.drinks = ['beer bottle' , 'fanta' , 'beer can' , 'coke' , 'seven up' , 'chocolate milk' , 'energy drink' , 'orange juice' , 'milk' , 'apple juice']
        self.places = ['start','living room','kitchen','dining room']
        self.word_list = ''
        self.word_recognized = ' '
        self.speech_talking = False
        
        # Vision Variables
        

        print "\nReady to work!!\n"

    def __service_waiting(self):

        lista = [
        '/semantic_map_server/load',
        '/semantic_map_server/get',
        '/goalServer/sendGoal',
        '/speech_synthesizer/synthesize',
        '/speech_recognizer/start',
        '/speech_recognizer/stop',
        '/speech_recognizer/load_dictionary'
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
        self.client_goal_set(destination_obj.semObj.pose,0)
        
        # Esperar llegada
        self.goal_reached = False
        while not self.goal_reached:
            time.sleep(0.5)
            
        return
    
    def approach_with_laser(self,distance=1):
        
        self.client_goal_approach(distance)
        
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
    
    def callback_speechRecognizer(self,data):
        
        print data.data
        print self.word_list
        
        if data.data in self.word_list:
            self.word_recognized = data.data
        else:
            self.word_recognized = 'none'
        
    def ask_question(self,question,dictionary):
        
        self.client_speech_load_dict(dictionary)
        
        if dictionary == 'confirmation':
            self.word_list = self.confirmation
        elif dictionary == 'names':
            self.word_list = self.names
        elif dictionary == 'drinks':
            self.word_list = self.drinks
        elif dictionary == 'places':
            self.word_list = self.places
        
        self.talk(question)
        self.client_speech_RecoStart()        
        
        while self.word_recognized == ' ' or self.word_recognized == 'none':
                
            if self.word_recognized == 'none':
                
                self.client_speech_RecoStop()
                self.talk('I did not understand')
                self.talk(question)
                self.client_speech_RecoStart()
                self.word_recognized = ' '
            else:
                time.sleep(1)
        self.client_speech_RecoStop()
        
        word_returned = self.word_recognized
        self.word_recognized = ' '
        return word_returned



if __name__ == '__main__':
    
    obj = utem_demo()
    obj.loadMap('utem_demo.sem_map')
    
#     print "Working"    
# 
#     print "Saludando"
#     # Hacer algun saludo
#     obj.talk("Hello, I am Bender the oocheela service robot")
#     
#     print "Preguntando por lugar"
#     # Ir hacia algun lugar que el publico elija
#     ##destination = obj.ask_question('where i must go?','places')
#     destination = 'dining room'
#     obj.talk('ok. i am going to the ' + str(destination))
#     print "Voy al " + str(destination)   
#     obj.setGoal(destination)
#     print "Llegue!!"
#     obj.talk("I reach the" + str(destination))
#     
#     # Ir a la cocina (siguiendo ciertos puntos intermedios)
#     obj.talk("Uff. All this walking around is making me feel thirsty")
#     obj.talk("i am going for a beverage")
#     obj.setGoal('pasillo_comedor')
    obj.setGoal('esquina_cocina')
    obj.setGoal('kitchen')
    print "En la cocina"

    # Ir a la posicion de grasping (verificar el lugar correcto)
    #obj.setGoal('cocina_toma_1') # Del 1 al 6
    obj.talk("approaching")
    obj.setGoal('pre_agarre')
    print "Approach with Laser!"
    #obj.approach_with_laser(0.3) # Quedar a 30 [cm]
    obj.talk("I reach the drink place")
    print "Llegue!!"

    # Hacer el grasping
    # - Preguntar que objeto tomar
    print "Preguntando"
#     selected_object =  obj.ask_question('What drink i should choose?','drinks')
#     print "Ok, i will lift the " + str(selected_object)
#     
    # - Tomar
    
    # - Algo_intermedio: Tal vez llevar el objeto a otro lugar
    # - Depositar

    # Ir a lugar de reposo
#    print "To the living room"
#     obj.talk("I am done here, so i am going to the living room")
#     #obj.setGoal('start')
#     
#     # Mensaje de despedida
#     obj.talk("See ya")         
