#!/usr/bin/env python

# R O S
import roslib; roslib.load_manifest('bender_behaviors')
import rospy

# P y t h o n
import sys
import math
import time

from std_msgs.msg import *
from std_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import *

rospy.init_node("demo_caras")

face = rospy.Publisher('/head',Emotion)
client_speech_synth = rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)

print "Listo para impresionar!"
while True:

    
    a = raw_input("Ingese una orden: ")

    if a == 'h1':
	face.publish("changeFace", "happy1", 0)
    elif a == "h2":
	face.publish("changeFace", "happy2", 0)
    elif a == "h3":
	face.publish("changeFace", "happy3", 0)
    elif a == "a1":
	face.publish("changeFace", "angry1", 0)
    elif a == "sur":
	face.publish("changeFace", "surprise", 0)
    elif a == "a2":
	face.publish("changeFace", "angry2", 0)
    elif a == "a3":
	face.publish("changeFace", "angry3", 0)
    elif a == "se":
	face.publish("changeFace", "serious", 0)
    elif a == "s1":
	face.publish("changeFace", "sad1", 0)
    elif a == "s2":
	face.publish("changeFace", "sad2", 0)
    elif a == "s3":
	face.publish("changeFace", "sad3", 0)
    elif a == "ash":
	face.publish("changeFace", "ashamed", 0)
    elif a == "fl":
	face.publish("changeFace", "flirt", 0)
    elif a == "eye":
	face.publish("changeFace", "eyebrow", 0)
    elif a == "mov":
	b = raw_input("X = ")
	face.publish("MoveX", "happy2", int(b))
   
    else:
	client_speech_synth(a)
   
