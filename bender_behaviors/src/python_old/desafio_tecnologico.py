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


rospy.init_node("desafio")

face = rospy.Publisher('/bender/face/head',Emotion)
client_speech_synth = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)

face.publish("MoveX", '', 0)

print "Presione Enter"
a = raw_input() # Espera un 'enter' para comenzar

face.publish("changeFace", "happy1", 0)
client_speech_synth("Hola desafiados")

time.sleep(2.5)
face.publish("changeFace", "happy2", 0)
client_speech_synth("soy b'ender")

time.sleep(3)

client_speech_synth("Les doy la bienvenida a los equipos en competencia")
time.sleep(5)

face.publish("changeFace", "serious", 0)
client_speech_synth("Saludamos a los alumnos del Complejo Educacional Juanita Fern'andez Solar de Recoleta")

time.sleep(7)
face.publish("changeFace", "1313", 0)
client_speech_synth("del Liceo n'umero uno Javiera Carrera de Santiago,")
time.sleep(4)

face.publish("changeFace", "happy3", 0)
client_speech_synth("y del British R'oyal Scul de La Reina.")
time.sleep(5)

face.publish("changeFace", "serious", 0)
client_speech_synth("Atenci'on")
time.sleep(2)

client_speech_synth("El desaf'io consistir'a en que deben construir y programar")
time.sleep(4.3)

face.publish("changeFace", "happy1", 0)
client_speech_synth("un veh'iculo robotizado que tiene que atravesar la ciudad")
time.sleep(5)

client_speech_synth("llevando materiales reciclables, pasando por seis check points,")

time.sleep(5.3)
face.publish("changeFace", "serious", 0)
client_speech_synth("hacia la planta de reciclaje.")

time.sleep(3.5)

client_speech_synth("Tienen una hora para construir el veh'iculo, cuatro horas para programarlo")
face.publish("changeFace", "happy2", 0)
time.sleep(6)

client_speech_synth("y tres oportunidades para ejecutar el desaf'io")

time.sleep(6)

client_speech_synth("ser'an evaluados por los jueces durante toda la jornada")

time.sleep(4.3)
face.publish("changeFace", "happy1", 0)
client_speech_synth("por la construcci'on del robot, por su trabajo en equipo, por la resoluci'on de problemas")

time.sleep(7.2)

client_speech_synth("y finalmente por la ejecuci'on en la arena.")
face.publish("changeFace", "happy2", 0)
time.sleep(5)
client_speech_synth("En cada puesto de trabajo asignado")

time.sleep(4)
client_speech_synth("encontrar'an materiales, herramientas de trabajo,")

time.sleep(4)
face.publish("changeFace", "serious", 0)
client_speech_synth("y las instrucciones detalladas de esta jornada.")

time.sleep(5)

client_speech_synth("Les deseo mucha suerte")
face.publish("changeFace", "happy3", 0)
time.sleep(2.5)

client_speech_synth("Y que comience el desaf'io tecnol'ogico!")

time.sleep(2)
face.publish("changeFace", "serious", 0)
face.publish("changeFace","happy3",0)