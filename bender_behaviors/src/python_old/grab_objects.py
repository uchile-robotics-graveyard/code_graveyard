#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *
import time

# Lista de palabras
objects = [ 'infusion', 'box' , 'glue', 'cup' , 'glass' ]
actions = [ 'lift', 'take', 'grab' ]
# Variables globales
speech_talking=False
word_recognized=' '
word_list=' '
debug=True
speechRecoStart = ''
speechRecoStop = ''
load_dict = ''
acc=' '

def talk(text):
    global speech_talking
    speechSynth= rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)
    speechSynth(text)
    print text
    speech_talking=False
    while not speech_talking:
        time.sleep(0.5)
    while speech_talking:
        time.sleep(0.5)

def grab_object():
    rospy.Subscriber('/speech_recognizer/output', String,speechRecognizerCallback)
    rospy.Subscriber('/speech_synthesizer/status', String,talkingCallback)

    print 'I am ready to go!'
    talk('what can i do for you?')
    accion=orden('actions')
    talk('what object you want me to '+ str(accion))
    objeto=orden('objects')

    talk('Ok, i will '+str(accion)+' the '+str(objeto))
    acc.publish(String(objeto))
    return [accion,objeto]

def orden(dictionary):
    global speechRecoStart, speechRecoStop, word_recognized,load_dict,word_list

    load_dict(dictionary)
    if dictionary == 'actions':
	word_list=actions
    elif dictionary == 'objects':
	word_list=objects

    speechRecoStart()

    print 'reco start'

    while word_recognized == ' ' or word_recognized == 'none':
        if word_recognized == 'none':
            speechRecoStop()
            talk('I did not understand')
            talk('can you repeat your request?')
            speechRecoStart()
            word_recognized = ' '
        else:	
            time.sleep(1)
    speechRecoStop()

    word_returned = word_recognized
    word_recognized = ' '
    return word_returned

# Callbacks
def talkingCallback(status):
    global speech_talking
    if status.data == 'Talking':
        speech_talking = True
    else:
        speech_talking = False

def speechRecognizerCallback(data):
    global word_recognized
    print data.data
    
    print word_list
    
    if data.data in word_list:
        word_recognized=data.data
    else:
        word_recognized='none'



def main():
    global speechRecoStart,speechRecoStop,load_dict,acc
    rospy.init_node('grab_object')

    acc = rospy.Publisher('/speech_arm_interface/accion',String) 
    
    speechRecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    
    grab_object()
        

if __name__ == '__main__':
    main()
