#!/usr/bin/env python
# encoding: utf-8

# R O S
import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

# P y t h o n 
import math
import time
from locale import str

from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import *

# Other python scripts
from entrance import button_state


class cocktail:
    
    def __init__(self):
        
        rospy.init_node('Cocktail Party')

        # Navigation stuff
        self.currentPose = PoseStamped()
        #self.goal_reached = asd

        # Speech stuff
        self.confirmation = ['yes' , 'no']
        self.names = ['michael' , 'jessica' , 'christopher' , 'ashley' , 'mathew' , 'brittany' , 'joshua' , 'amanda' , 'daniel' , 'samantha' , 'david' , 'sarah' , 'andrew' , 'stephanie' , 'james' , 'jennifer' , 'justin' , 'elizabeth' , 'joseph' , 'lauren']
        self.drinks = ['beer bottle' , 'fanta' , 'beer can' , 'coke' , 'seven up' , 'chocolate milk' , 'energy drink' , 'orange juice' , 'milk' , 'apple juice']
        self.speech_talking=False
        self.load_dict = ''
        self.speechRecoStart = ''
        self.speechRecoStop = ''
        self.word_recognized=' '
        self.word_list=' '

        # Vision stuff
        self.lastWave = WaveData()
        self.lastWaveFlag = False

    # N A V I G A T I O N    F U N C T I O N S
    def poseCallback(self,data):
        self.currentPose.pose = data.pose.pose

    def goalFBcallback(self,data):
        self.goal_reached = (data.data == "Goal Reached")
        print data

    def goalWithStopButton(self,destination):
        
        # TODO: ??
        global button_state
        
        #init
        semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
        rospy.wait_for_service('/semantic_map_server/get')
        setGoal=rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
        
        print "al !"+destination
        livingPose=semCaller(destination)
        setGoal(livingPose.semObj.pose,0)
        time.sleep(0.5)
        
        #esperar a que llegue con boton de emegencia
        self.goal_reached = False
        while not self.goal_reached:
            if button_state:
                cancelGoal()
                while button_state:
                    time.sleep(0.5)
                setGoal(livingPose.semObj.pose,0)
                time.sleep(1)
            time.sleep(0.5)
        return

    def lookToPose(self,ID):
        global button_state
        
        semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
        rospy.wait_for_service('/semantic_map_server/get')
        lookSrv = rospy.ServiceProxy('/goalServer/lookToPoseStamped', lookToPoseStamped)
        rospy.wait_for_service('/goalServer/lookToPoseStamped')
        
        print "lookin to !"+ID
        id_pose=semCaller(ID)
        
        id_poseStamped = PoseStamped()
        id_poseStamped.header.frame_id = "/map"
        id_poseStamped.header.stamp = rospy.Time.now()
        id_poseStamped.pose = id_pose.semObj.pose
            
        lookSrv(id_poseStamped)
        time.sleep(0.5)
        
        #esperar a que llegue con boton de emegencia
        self.goal_reached=False
        while not self.goal_reached: 
            if button_state:
                cancelGoal()
                while button_state:self.speech_talking = False
                    time.sleep(0.5)
                lookSrv(id_poseStamped)
                time.sleep(1)
            time.sleep(0.5)
        return

    def goToPerson(self,x_,y_):
        global button_state
        
    
        print 'GO TO PERSON'
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "/map"
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose.position.x = x_
        poseStamped.pose.position.y = y_
        poseStamped.pose.orientation.w = 1.0 
            
        goSrv = rospy.ServiceProxy('/goalServer/irPersona', lookToPoseStamped)
        rospy.wait_for_service('/goalServer/irPersona')
        
        goSrv(poseStamped)
        time.sleep(0.5)
        
        #esperar a que llegue con boton de emegencia
        self.goal_reached=False
        while not self.goal_reached:
            if button_state:
                cancelGoal()
                while button_state:
                    time.sleep(0.5)
                goSrv(poseStamped)
                time.sleep(1)
            time.sleep(0.5)
        return


    # S P E E C H    F U N C T I O N S
        
    def ask_question(self,question,dictionary):
        
        load_dict(dictionary)
        
        if dictionary == 'confirmation':
            self.word_list = confirmation
        elif dictionary == 'names':
            self.word_list = names
        elif dictionary == 'drinks':
            self.word_list = drinks
        
        talk(question)
        speechRecoStart()
        
        print 'reco start'
        
        while self.word_recognized == ' ' or self.word_recognized == 'none':
            if self.word_recognized == 'none':
                speechRecoStop()
                talk('I did not understand')
                talk(question)
                speechRecoStart()
                self.word_recognized = ' '
            else:
                time.sleep(1)
        speechRecoStop()
        
        word_returned = self.word_recognized
        self.word_recognized = ' '
        return word_returned


    def speechRecognizerCallback(self,data):
        
        print data.data
        print self.word_list
        
        if data.data in self.word_list:
            self.word_recognized = data.data
        else:
            self.word_recognized = 'none'

    def interaccion_cocktail(self):
        rospy.Subscriber('/speech_recognizer/output', String, speechRecognizerCallback)
        rospy.Subscriber('/speech_synthesizer/status', String,talking_callback)
        
        print 'ready'
        
        name = ask_question('what is your name?','names')
        
        drink = ask_question('what drink do you want?','drinks')
        
        talk('Ok ' + str(name)+' i will bring you a '+str(drink))
        
        return [name,drink]


def main():
    global speechRecoStart,speechRecoStop,load_dict
    
    rospy.init_node('cocktail_party')
    
    speechRecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    
    pub = rospy.Publisher('/cmd_vel', Twist)
    order = Twist()
    
    goalFB = rospy.Subscriber('/goalServer/feedback', String, goalFBcallback)
      

    
    #dictionaryLoader = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    #print "waiting speech"
    #rospy.wait_for_service('/speech_recognizer/load_dictionary')

    #dictionaryLoader("cocktail")
    
    #speechSynthState=rospy.Subscriber('/speech_synthesizer/status',String,speechSynthStatusCallback)
    waveKinect=rospy.Subscriber('/WaveUserData',WaveData,waveCallback)
    pose_sub=rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,poseCallback)
    kinect_motor=rospy.Publisher('/tilt_angle',Float64)
    


    print "working"
    talk("Ready to work")    
    

    print"al living"
    goalWithStopButton("living room")

    #Presentacion
    print "Looking for people"
    talk("Looking for people calling me")
    
    while True:
        
        listo = False
        poseArray = ["c1","c2","c3","c4","c5"]
        x=waitForWave(7)
        if len(x)!=0:
            listo = True
        
        count = 0
        
        while not listo:
            
            lookToPose(poseArray[count])
            x=waitForWave(7)    
            print x
            
            if len(x)!=0:
                listo = True
            else:
                talk("No one is calling me there")
                
            count+=1
            if count==5:
                count=0
            
        
        
        talk("I see someone calling me")#+str(round(x[0].pose.position.x*100)/100))
        talk("Aproaching to him")
            
        theta = 2.0*math.atan2(currentPose.pose.orientation.z, currentPose.pose.orientation.w)
        x__ = currentPose.pose.position.x + math.cos(theta)*x[0].pose.position.x
        y__ = currentPose.pose.position.y + math.sin(theta)*x[0].pose.position.y 
            
        goToPerson(x__,y__)
        
        #TALK
        interaccion_cocktail()
        time.sleep(5)
        goalWithStopButton("bar")
        
if __name__ == '__main__':
    main()

