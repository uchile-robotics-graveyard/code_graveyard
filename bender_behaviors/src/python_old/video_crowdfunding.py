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

# Services
from std_srvs.srv import *

from bender_msgs.msg import *
from bender_srvs.srv import *


rospy.init_node("ideame")

#face = rospy.Publisher('/bender/face/head',Emotion)
client_speech_synth = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)

#face.publish("MoveX", '', 0)

print "Presione Enter"
a = raw_input() # Espera un 'enter' para comenzar

#face.publish("changeFace", "happy1", 0)
client_speech_synth("Hola amigos, Mi nombre es B'ender")

time.sleep(4)
#face.publish("changeFace", "happy2", 0)
client_speech_synth("soy un robot de servicio")

time.sleep(3)

client_speech_synth("creado por los alumnos de ingenier'ia el'ectrica de la universidad de Chile")
time.sleep(5)

#face.publish("changeFace", "serious", 0)
client_speech_synth("Desde el 2007 participo en la competencia internacional robocup")
time.sleep(5)

#face.publish("changeFace", "serious", 0)
client_speech_synth("donde participan robots de todo el mundo y soy unico representante de latinoamerica.")

time.sleep(7)
#face.publish("changeFace", "1313", 0)
client_speech_synth("He obtenido el premio a la innovacion en dos oportunidades")
time.sleep(4)

#face.publish("changeFace", "happy3", 0)
client_speech_synth("y he estado dentro 5 primeros lugares a nivel mundial")
time.sleep(5)

#face.publish("changeFace", "serious", 0)
client_speech_synth("este anio la competencia es en china por lo que conocere un nuevo y asombroso lugar")
time.sleep(2)

client_speech_synth("para competir con los mejores robots del mundo")
time.sleep(4.3)

#face.publish("changeFace", "happy1", 0)
client_speech_synth("lamentablemente viajar a china es muy costoso y mi equipo")
time.sleep(5)

client_speech_synth("no cuenta con los recursos para ayudarme y viajar conmigo")

time.sleep(5.3)
#face.publish("changeFace", "serious", 0)
client_speech_synth("asi que decidi darles una mano y grabar este video ")

time.sleep(3.5)

client_speech_synth("solicitando la ayuda de todos ustedes.")
#face.publish("changeFace", "happy2", 0)
time.sleep(6)

client_speech_synth("aporta en ideame y ayuda a bender a llegar a china.")

time.sleep(6)
