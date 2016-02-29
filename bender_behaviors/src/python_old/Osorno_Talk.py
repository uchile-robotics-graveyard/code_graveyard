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


rospy.init_node("ceremonia")

face = rospy.Publisher('/head',Emotion)
client_speech_synth = rospy.ServiceProxy('/speech_synthesizer/synthesize',synthesize)

print "Presione Enter"
a = raw_input() # Espera un 'enter' para comenzar

client_speech_synth("Pero senior rector que esta diciendo ")
face.publish("changeFace", "sad1", 0)
time.sleep(3)

client_speech_synth("creo que esta equivocado")
face.publish("changeFace", "angry1", 0)
time.sleep(6)

client_speech_synth("Muchas gracias")

time.sleep(3)

print "Presione Enter para continuar"
a = raw_input() # Espera un 'enter' para comenzar

client_speech_synth("Mi nombre es Bender y soy el robot de servicio")
face.publish("changeFace", "serious", 0)
time.sleep(3.5)

client_speech_synth("del laboratorio de rob'otica de la Universidad de Chile")

time.sleep(6)

client_speech_synth("Fui creado el anio dos mil siete por alumnos de ingiener'ia el'ectrica")

time.sleep(6)

client_speech_synth("Participo en los mundiales de rob'otica llamados robocap")

time.sleep(6)

client_speech_synth("El mejor lugar que he obtenido es el quinto lugar de 24 equipos")

time.sleep(7)

client_speech_synth("Obtuve el premio a la innovaci'on el anio dos mil siete y dos mil ocho")

time.sleep(6.5)

client_speech_synth("Adem'as he dado charlas a distintos colegios y universidades  en santiago y otras ciudades de Chile")

time.sleep(9)

client_speech_synth("Tengo muchas habilidades")

time.sleep(3)

client_speech_synth("puedo reconocer rostros y voces")

time.sleep(4)

client_speech_synth("tambi'en puedo detectar y manipular objetos con mis brazos de fibra de carbono")

time.sleep(6)

client_speech_synth("puedo navegar por espacios desconocidos y hacer mapas")

time.sleep(6)

client_speech_synth("puedo mostrar mis emociones")

time.sleep(4)
client_speech_synth("como tristeza")
face.publish("changeFace", "sad2", 0)
time.sleep(5)
client_speech_synth("o felicidad")
face.publish("changeFace", "happy3", 0)
time.sleep(6)
face.publish("changeFace", "serious", 0)
client_speech_synth("Los invito a ver mi estand para las demostraciones de mis habilidades")

time.sleep(6)

client_speech_synth("Tambi'en pueden ver a mis amigos que juegan futbol")

time.sleep(8)

client_speech_synth("Lo invito senior rector a continuar con su discurso")

time.sleep(5)
