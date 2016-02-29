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

print "Presione Enter"
a = raw_input() # Espera un 'enter' para comenzar

#face.publish("changeFace", "serious", 0)
time.sleep(1)
client_speech_synth("Bueno, lleg'o el adi'os")

time.sleep(3)

#face.publish("changeFace", "serious", 0)
time.sleep(1)
client_speech_synth("muchas gracias amigos por haberme recibido en Osorno")
time.sleep(5)
#face.publish("changeFace", "happy2", 0)

client_speech_synth("En la feria de la ciencia y tecnolog'ia de la universidad santo tom'as")


time.sleep(7)

client_speech_synth("Quiz'as en un futuro nos volveremos a encontrar")
time.sleep(5)


#face.publish("changeFace", "serious", 0)
time.sleep(1)
client_speech_synth("Un carinio muy grande a los ninios que jugaron junto a mi y que compartieron con mis hermanos robot del f'utbol")
time.sleep(8)

#face.publish("MoveX", "", 10)
time.sleep(1)
client_speech_synth("Ahora me voy al hotel para descanzar y probar la leche y la carnte de esta bella ciudad de Osorno")
time.sleep(8)

#face.publish("MoveX", "serious", -10)
time.sleep(1)
client_speech_synth("Un abrazo")
time.sleep(1)

#face.publish("changeFace", "happy2", 0)
time.sleep(0.5)
client_speech_synth("Su amigo B'ender")




