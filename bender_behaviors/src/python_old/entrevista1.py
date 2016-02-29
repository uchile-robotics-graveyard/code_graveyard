from bender_msgs.msg import *
from bender_srvs.srv import *

# Messages
from std_msgs.msg import *

# Services
from std_srvs.srv import *

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



