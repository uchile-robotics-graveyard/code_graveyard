#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *
import time

#definiciones
confirmation=['yes','no']
names= [ 'michael' , 'jessica' , 'christopher' , 'ashley' , 'mathew' , 'brittany' , 'joshua' , 'amanda' , 'daniel' , 'samantha' , 'david' , 'sarah' , 'andrew' , 'stephanie' , 'james' , 'jennifer' , 'justin' , 'elizabeth' , 'joseph' , 'lauren']
drinks= ['beer bottle' , 'fanta' , 'beer can' , 'coke' , 'seven up' , 'chocolate milk' , 'energy drink' , 'orange juice' , 'milk' , 'apple juice']

#variables globales
speech_talking=False
word_recognized=' '
word_list=' '
debug=True

speechRecoStart = ''
speechRecoStop = ''
load_dict = ''

#funciones auxiliares
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
        
def ask_question(question,dictionary):
    global word_recognized,word_list
    
    load_dict(dictionary)
    
    if dictionary == 'confirmation':
        word_list=confirmation
    elif dictionary == 'names':
        word_list=names
    elif dictionary == 'drinks':
        word_list=drinks
    
    talk(question)
    speechRecoStart()
    
    print 'reco start'
    
    while word_recognized == ' ' or word_recognized == 'none':
        if word_recognized == 'none':
            speechRecoStop()
            talk('I did not understand')
            talk(question)
            speechRecoStart()
            word_recognized = ' '
        else:
            time.sleep(1)
    speechRecoStop()
    
    
    word_returned = word_recognized
    word_recognized = ' '
    return word_returned

#callbacks
def talking_callback(status):
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

def interaccion_cocktail():    
    rospy.Subscriber('/speech_recognizer/output', String,speechRecognizerCallback)
    rospy.Subscriber('/speech_synthesizer/status', String,talking_callback)
    
    print 'ready'
    
    name = ask_question('what is your name?','names')
    
    drink = ask_question('what drink do you want?','drinks')
    
    talk('Ok '+str(name)+' i will bring you a '+str(drink))
    
    return [name,drink]

def main():
    global speechRecoStart,speechRecoStop,load_dict
    rospy.init_node('interaccion_cocktail')
    
    
    speechRecoStart = rospy.ServiceProxy('/speech_recognizer/start',Empty)
    speechRecoStop = rospy.ServiceProxy('/speech_recognizer/stop', Empty)
    load_dict = rospy.ServiceProxy('/speech_recognizer/load_dictionary', load_dictionary_service)
    
    interaccion_cocktail()
    

if __name__ == '__main__':
    main()
