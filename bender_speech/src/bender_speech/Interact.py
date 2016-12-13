#!/usr/bin/env python

import roslib
import rospy
import time
import math
import smach
import smach_ros
from std_srvs.srv import Empty
from std_msgs.msg import String as std_String
from bender_srvs.srv import TimerString, TimerStringResponse
from bender_macros.speech import Talk  
from bender_utils.ros import benpy
from bender_ai import AI
from datetime import datetime
import os

class Interact_server():

    def __init__(self):

        self.bender_ai = AI.RobotAI()
        
        self.aiml_dir = roslib.packages.get_pkg_dir('bender_ai')+'/aiml'
        #self.bender_ai.initialize(self.aiml_dir)

        # ROS communication
        self.s = rospy.Service('~/answer_request',TimerString,self.answer_request_server)
        self.start_reco_client = rospy.ServiceProxy('/bender/speech/recognizer/start', Empty)
        self.stop_reco_client  = rospy.ServiceProxy('/bender/speech/recognizer/stop', Empty)
        #self.start_reco_client2 = rospy.ServiceProxy('/bender/speech/recognizer2/start', Empty)
        #self.stop_reco_client2  = rospy.ServiceProxy('/bender/speech/recognizer2/stop', Empty)
        rospy.Subscriber('/bender/speech/recognizer/output', std_String, self._request_callback)
        #rospy.Subscriber('/bender/speech/recognizer2/output', std_String, self._request_callback)
        self.pub = rospy.Publisher('/bender/speech/recognizer/output', std_String, queue_size=10)

        self.words = dict()
        self.sepWords()
        self.question = ''
        self.cont = 0

    def _request_callback(self,msg):
        self.question = self.modString(msg.data)
        #print 'Pregunta: '+msg.data+' -> '+self.question

    def answer_request_server(self,req):
        
        self.pub.publish(' ')
        rospy.sleep(0.5)
        try:
            #self.start_reco_client2() if req.data == 'indirect' else self.start_reco_client()
            self.start_reco_client()
        except rospy.ServiceException, e:
            rospy.logerr("ups. failed to load or start speech recognition. " + str(e))
        
        self.cont = req.timeout
        
        # se pega req.timeout segundos escuchando al microfono
        
        while True:
            
            if self.listen_question(req) == -1:
                return TimerStringResponse('',True)
            elif self.listen_question(req) == 1:
                rospy.loginfo('I heard: '+self.question)
    
                # si escucha algo coherente busca respuesta
                if self.find_answer(req) == 1:
                    return TimerStringResponse(self.response,False)
                elif self.find_answer(req) == -1:
                    pass

    def listen_question(self,req):
        while self.question == '':
            rospy.loginfo('I still do not hear a question, '+str(self.cont)+' seconds left...')
            rospy.sleep(1)
            if len(self.question) < 10 and len(self.question) > 0:
                rospy.loginfo('I heard a noise: "'+self.question+'": '+str(len(self.question))+' words')
                self.question = ''
            if self.cont == 0:
                #self.stop_reco_client2() if req.data == 'indirect' else self.stop_reco_client()
                self.stop_reco_client()
                self.question=''
                return -1
            self.cont-=1
        
        return 1

    def find_answer(self, req):
        #print self.question
        self.r = self.bender_ai.getResponse(self.question)
        if self.r != '':
            if self.r == 'time':
                self.answer="It is %s"%datetime.now().strftime("%I %M %p")
            else:
                self.answer=self.r
            
            self.response=self.question+':'+self.answer
            #self.stop_reco_client2() if req.data == 'indirect' else self.stop_reco_client()
            self.stop_reco_client()
            self.question=''
            return 1
        else:
            self.question=''
            return -1

    def sepWords(self):
        pkg_dir = roslib.packages.get_pkg_dir('bender_utils')
        words_file = open(os.sep.join([pkg_dir,'/config/mapper/speechrecogparser']),'r')
        for line in words_file:
            #print line
            parts = line.split(':')
            self.words[parts[0]] = parts[1].replace('\n','')
    
    def modString(self, input):
        
        output = ''
        line_words = input.split(' ')
        #print line_words
        for word in line_words:
            #print word
            #print self.words.keys()
            if word in self.words.keys():
                new_word = self.words[word]
                output+=new_word+' '
            else:
                output+=word+' '
        
        return output

def main(): 

    Interact_server()
    rospy.spin()

# TODO: maquinas de estado

if __name__ == '__main__':

    rospy.init_node('interact')

    main()
