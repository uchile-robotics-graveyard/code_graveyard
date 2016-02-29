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

face = rospy.Publisher('bender/face/head',Emotion)
client_speech_synth = rospy.ServiceProxy('bender/speech/synthesizer/synthesize',synthesize)


while True:

    print "Presione Enter"
    a = raw_input()

    face.publish("MoveX", "happy1", 10)
    client_speech_synth("Hola Amigos")

    face.publish("changeFace", "happy1", 0)
    time.sleep(3)
    client_speech_synth("como estan")
    face.publish("changeFace", "happy2", 0)
    time.sleep(4)
    face.publish("MoveX", "happy1", -20)

    client_speech_synth("Estoy muy feliz de conocerlos")
    face.publish("changeFace", "happy3", 0)
    time.sleep(5)
    client_speech_synth("Soy bender")
    face.publish("changeFace", "happy2", 0)
    time.sleep(3)
    client_speech_synth("el robot de servicio de la Universidad de Chile")
    time.sleep(5)
    face.publish("MoveX", "happy1", -20)
    time.sleep(5)
    client_speech_synth("Seguramente me conocen")
    face.publish("MoveX", "happy1", -10)
    time.sleep(3)

    client_speech_synth("porque puedo mostrar mis emociones")
    time.sleep(3)

    client_speech_synth("si me molestanme pongo triste")
    face.publish("changeFace", "sad1", 0)
    time.sleep(3)
    face.publish("changeFace", "sad2", 0)
    time.sleep(3)
    face.publish("changeFace", "sad3", 0)


    client_speech_synth("o me puedo enojar")
    time.sleep(3)
    face.publish("changeFace", "angry1", 0)
    time.sleep(3)
    face.publish("changeFace", "angry2", 0)

    face.publish("changeFace", "angry3", 0)

    client_speech_synth("casi siempre estoy serio")
    time.sleep(5)
    face.publish("changeFace", "serious", 0)
    time.sleep(3)

    client_speech_synth("pero me sorprendo facilmente")
    time.sleep(5)
    face.publish("changeFace", "surprise", 0)


    time.sleep(3)
    face.publish("changeFace", "eyebrow", 0)
    time.sleep(3)
    face.publish("changeFace", "flirt", 0)
    client_speech_synth("aunque la mayoria del tiempo estoy feliz")
    face.publish("MoveX", "happy1", -20)
    time.sleep(3)
    face.publish("changeFace", "happy3", 0)
    time.sleep(6)
    face.publish("MoveX", "happy1", -1,	0)
    face.publish("changeFace", "serious", 0)

    print "F I N"


