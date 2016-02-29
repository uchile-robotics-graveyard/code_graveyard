#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_behaviors')
import rospy
import math
import smach
import smach_ros
import cv2
import time

# Para el algoritmo Binary_search <3
from bisect import bisect_left

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

# - - - - -macros- - - - -

from bender_behaviors.Stage1.GPSR import Pick
from bender_behaviors.Stage2.WakeMeUpTest import ParseWake
from bender_macros.arm import GraspCartesian
from bender_macros.arm import PlaceObjectPose

from bender_macros.nav import ApproachToPoseStamped
from bender_macros.nav import GotoPlace
from bender_macros.nav import RoomEntering
from bender_macros.nav import ApproachtoTable
from bender_macros.nav import LooktoPoseStamped

from bender_macros.knowledge import PoseMapSetup

from bender_macros.vision import FindObject
from bender_core import benpy

from bender_macros.speech import Talk
from bender_macros.speech import AskForConfirmation
from bender_macros.speech import Recognize



class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['succeeded','aborted','preempted'],
                             io_keys = ['placing_array'],
                             output_keys = ['setmap','setplace','dictionary','object_counter',
                             'table','bftable','cereal_array','setbed','setkitchen',
                             'placing_pos','cereal_pos'])
    
    def execute(self,userdata):
        # Definir mapa 
        userdata.setmap = ""

        # Definir lugares (cambiar por variables reales en el mapa)
        userdata.setplace = "room"
        userdata.setbed = "bed"
        userdata.setkitchen = "kitchen"
        userdata.table = "kitchentable" #CAMBIAR ESTA VARIABLE POR NOMBRE DE MESA EN EL MAPA!!
        userdata.bftable = "servingtable" #'' '' '' ' '' '' ' '' !!

        # Posicion objetos de desayuno en mesa
        userdata.placing_array = [[0.0 , 0.1, 0.3 ], [ 0.0, 0.1, 0.3 ] , [ 0.0, 0.1, 0.3 ]]
        #Arreglo de posiciones
        # que usa la funcion PlaceObjectPose y por consiguiente la funcion setPosition de la macro ArmStates
        userdata.placing_pos = [0.0, 0.1, 0.3]

        # Cereal Pose
        userdata.cereal_pos = [0.0, 0.1, 0.3]

        # Diccionario
        userdata.dictionary = "" #loaddictionary

        # Arreglo de posibilidades de cereales
        userdata.cereal_array = ['zukosos','zukaritas','zhukulento']

        # Contador de objectos
        userdata.object_counter = 0

        # Transform placing array to map
        # userdata.placing_array = transform_toMap(userdata.placing_array)

        return 'succeeded'

class Person_Identify(smach.State):
    def __init__(self):       
        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'],
                             input_keys=['in_data'],
                             output_keys=['out_data'])
                             
    def execute(self,userdata):
        #Identificar a la persona en la cama para que Bender
        #pueda detectar sus cambios de posicion (de acostado a sentado)
        #tambien rostro en caso de poder detectar si abre los ojos
        #entre otras ideas
        rospy.loginfo('Identifying Person')
        return 'succeeded'

#Esta es la alarma para despertar al individuo
class AlarmMode(smach.State):
    def __init__(self):       
        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted','again'])
        self.t0 =time.time()
        rospy.loginfo('Turning On Alarm Mode')
        self.sound = benpy.ServiceProxy('/bender/fun/sound_player/play',play_sound)

    def execute(self,userdata):
        person_state = 0 #El estado es 0 para dormido y 1 para despierto,
        #remover mas tarde
        while (time.time()-self.t0)<60: #la funcion while que verifica el tiempo
            time.sleep(1)
            rospy.loginfo(time.time()-self.t0) 
            if ((time.time()-self.t0) % 10 < 0.6 or (time.time()-self.t0)<1.5 ):
                try:
                    self.sound(1, 'clock_alarm')
                    rospy.loginfo("Sonido alarma")
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                    return 'aborted'
            if rospy.is_shutdown() is True:
                return 'preempted'
            elif person_state == 0:
                rospy.loginfo('Alarm Up')
            elif person_state == 1:
                rospy.loginfo('Awakening Succesful')
                return 'succeeded'
            else:
                return 'preempted'
        return 'aborted'
#Esta  es el estado en el cual la persona se asume despierta y se le
#pregunta su desayuno
class Asking_for_breakfast(smach.State):    
    def __init__(self):       
        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'],
                            io_keys = ['counter'])

    def execute(self,userdata):
        Talk.getInstance('Good Morning',2)
        rospy.loginfo('Asking for breakfast')
        Talk.getInstance('What do you want for breakfast?',9)
        if userdata.counter < 3:
            userdata.counter += 1
            rospy.loginfo('Try number: '+str(userdata.counter))
            return 'succeeded'
        else:
            rospy.loginfo('No more tries')
            return 'aborted'

class Object_Processor(smach.State):    
    def __init__(self):       
        smach.State.__init__(self,outcomes=['succeeded','aborted','done','serve_bowl'],
                            io_keys = ['object_name','bowl_status','placing_pos'] ,
                            input_keys = ['object_array','object_counter','placing_array'])

    def execute(self,userdata):
        rospy.loginfo( str(userdata.object_array) )

        if userdata.object_name == 'bowl' and userdata.bowl_status == 0: # Unplaced
                rospy.loginfo('Placing Bowl')
                userdata.bowl_status = 1
                userdata.placing_pos = userdata.placing_array[2] # Esto debe ponerse en la sm de cuando se va a dejar el bowl :c
                return 'serve_bowl'

        if userdata.object_counter < len(userdata.object_array):
            userdata.object_name = userdata.object_array[userdata.object_counter]
            userdata.placing_pos = userdata.placing_array[userdata.object_counter]
            rospy.loginfo('Object to pick is ' + userdata.object_name)

            return 'succeeded'
        else:
            rospy.loginfo('All objects served')
            return 'done'

        return 'aborted'
    
class ObjectinHand(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['succeeded','serve_cereal','preempted'],
                                input_keys = ['cereal_array','object_pose'],
                                io_keys = ['object_name','object_counter','cereal_pos'])
        
    def execute(self,userdata):
        
        sorted_array = self.sort(userdata.cereal_array)

        if (userdata.object_name or userdata.cereal_array) is None:
            return 'preempted'
           
        if self.binary_search( sorted_array, userdata.object_name, 0, None) != -1:
        
            userdata.object_counter += 1
            rospy.loginfo('On my way to serve the cereal')
            userdata.cereal_pos = transform_toMap(userdata.object_pose)
            return 'serve_cereal'

        userdata.object_counter += 1

        return 'succeeded'

    def binary_search(self, a, x, lo = 0, hi = None):   
        hi = hi if hi is not None else len(a)    
        pos = bisect_left(a,x,lo,hi)         
        return (pos if pos != hi and a[pos] == x else -1) 

    def sort(self, array):
        less = []
        equal = []
        greater = []

        if len(array) > 1:
            pivot = array[0]
            for x in array:
                if x < pivot:
                    less.append(x)
                if x == pivot:
                    equal.append(x)
                if x > pivot:
                    greater.append(x)
            # Don't forget to return something!
            return self.sort(less) + equal + self.sort(greater)  # Just use the + operator to join lists
        # Note that you want equal ^^^^^ not pivot
        else:  # You need to hande the part at the end of the recursion - when you only have one element in your array, just return the array.
            return array


            
#Estado final para marcar el termino de el Test completo
class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['succeeded','aborted','preempted'])
    def execute(self,userdata):
        rospy.loginfo('Test done')
        return 'succeeded'
        
              

def transform_toMap(poses):
    
        transformer = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)

        poses_transformer = poses
        print "found " + str(len(poses))
        cont = 0
        for i in poses:
            try:
                req = TransformerRequest()

                req.pose_in = i
                req.frame_out = "map"
                transf_out = transformer(req)
                ps = transf_out.pose_out
                poses_transformer[cont] = ps

                cont += 0

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        return poses_transformer

        
def getInstance():
    rospy.init_node('wakemeup')
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # important data
    sm.userdata.right_arm = '/right_arm'
    sm.userdata.counter = 0
    sm.userdata.bowl_status = 0 # Unplaced

    # Dummy userdata
    sm.userdata.object_name = ""
    sm.userdata.placing_array = []
    with sm:
        
        smach.StateMachine.add('Setup',Setup(),
            transitions = {'succeeded':'Room_entrance'}  
        )

        smach.StateMachine.add('Room_entrance',GotoPlace.getInstance(),
            transitions = {'succeeded':'MoveToBed','preempted':'Room_entrance',
                            'aborted':'MoveToBed','again':'Room_entrance'},
            remapping = {'map_name':'setmap', 'place_name':'setplace'}
        )
        
        smach.StateMachine.add('ApproachtoBed',ApproachToPoseStamped.getInstance(),
            transitions = {'succeeded':'Person_Identify','preempted':'ApproachtoBed'},
            remapping = {'map_name':'setmap','place_name':'setbed'}
        )

        smach.StateMachine.add('Person_Identify',Person_Identify(),
            transitions = {'succeeded':'AlarmMode'}
        )
        
        smach.StateMachine.add('AlarmMode',AlarmMode(),
            transitions = {
                'succeeded':'Asking_for_breakfast',
                'again':'AlarmMode',
                'aborted':'Asking_for_breakfast',
                'preempted':'AlarmMode'
            }
        )
        
        smach.StateMachine.add('Asking_for_breakfast',Asking_for_breakfast(),
            transitions = {
                'succeeded':'Recognize_breakfast',
                'preempted':'Asking_for_breakfast',
                'aborted':'aborted'
            },
            remapping = {'counter':'counter'}
        )
        
        smach.StateMachine.add('Recognize_breakfast',Recognize.getInstance(),
            transitions = {
                'succeeded':'ConfirmBreakfast',
                'aborted':'Recognize_breakfast',
                'preempted':'Asking_for_breakfast'
            },
            remapping = {'dictionary':'dictionary'}
        )

        smach.StateMachine.add('Food_filter', ParseWake.getInstance(),
            transitions = {'succeeded':'ConfirmBreakfast'},
            remapping = {'recognized_phrase':'recognized_word'}
        ) 
        
        smach.StateMachine.add('ConfirmBreakfast',AskForConfirmation.getInstance(),
            transitions = {
                'succeeded':'ConfirmBreakfast',
                'aborted':'ConfirmBreakfast',
                'yes':'Object_Processor',
                'no':'Asking_for_breakfast'
            },
            remapping = {'text':'breakfast_order','timeout':'breakfast_timeout'}
        )
        
        smach.StateMachine.add('Object_Processor',Object_Processor(),
            transitions = {
                'succeeded':'GotoKitchen',
                'aborted':'Object_Processor',
                'done':'Finish',
                'serve_bowl':'Object_Processor'
            },
            remapping = {
                'object_counter':'object_counter',
                'object_name':'object_name',
                'object_array':'object_array',
                'bowl_status':'bowl_status',
                'placing_pos':'placing_pos',
                'placing_array':'placing_array'
            }
        )

        smach.StateMachine.add('GotoKitchen',GotoPlace.getInstance(),
            transitions = {'succeeded':'GotoTable','preempted':'GotoKitchen'},
            remapping = {'map_name':'setmap','place_name':'setkitchen'}
        )

        smach.StateMachine.add('GotoTable',ApproachtoTable.getInstance(),
            transitions = {'succeeded':'LooktoTable','aborted':'GotoTable'},
            remapping = {'map_name':'setmap','table_name':'table'}
        )

        smach.StateMachine.add('LooktoTable',LooktoPoseStamped(),
            transitions = {'succeeded':'LookForObject'},
            remapping = {'map_name':'setmap','place_name':'table'}
        )

        smach.StateMachine.add('LookForObject',FindObject.getInstance(),
            transitions = {
                'succeeded':'PickfromTable',
                'aborted':'LookForObject',
                'not_found':'LookForObject'
            },
            remapping = {'required_object':'object_name'}
        )

        smach.StateMachine.add('PickfromTable',GraspCartesian.getInstance(),
            transitions = {'succeeded':'Taketo_bfTable','preempted':'PickfromTable','aborted':'aborted','notGrabbed':'PickfromTable'},
            remapping = {'selected_position':'object_pose','selected_arm':'right_arm'}
        )
        
        # # # Cereal Pose required
        smach.StateMachine.add('ReturntoPose_Cereal',ApproachToPoseStamped.getInstance(),
            transitions = {'succeeded':'Leave_Object'},
            remapping = {'map_name':'setmap','place_name':'cereal_pos'}
        )

        smach.StateMachine.add('Taketo_bfTable',ApproachToTable.getInstance(),
            transitions = {'succeeded':'ObjectinHand','preempted':'Object_Processor','aborted':'Taketo_bfTable'},
            remapping = {'map_name':'setmap','place_name':'bftable'}
        )   #preempted; si cae el objeto

        # # Estado que revisa si tiene el cereal en mano, en caso de que si el estado se dirigira a servirlo
        # #  en el caso contrario procedera simplemente a soltarlo
        smach.StateMachine.add('ObjectinHand',ObjectinHand(),
            transitions = {'succeeded':'Object_Processor','serve_cereal':'ServeCereal','preempted':'ObjectinHand'},
            remapping = {'cereal_array':'cereal_array','object_name':'object_name',
                         'object_counter':'object_counter','cereal_pos':'cereal_pos',
                         'object_pose':'object_pose'}
        )

        smach.StateMachine.add('ServeCereal',ServeCereal(),
            transitions = {'succeeded':'ReturntoPose_Cereal','aborted':'ServeCereal'},
            remapping = {'bowl_position':'placing_pos'}
        )

        smach.StateMachine.add('Leave_object',PlaceObjectPose.getInstance(),
            transitions = {'succeeded':'Object_Processor','preempted':'Object_Processor','aborted':'Leave_object'},
            remapping = {'lr_arm':'right_arm','pos_in_table':''}
        )
                            
        smach.StateMachine.add('Finish', Finish(),
            transitions = {'succeeded':'succeeded'}
        )
        
    
        initial_data = smach.UserData()
        # initial_data.object_array = ['arroz','leche con chocolate','zucosos remojados','sushi','ensalada','helado','poi']
        # initial_data.object_counter = 0
        # initial_data.cereal_array = ['cachana','amarillo','zucosos remojados','pasto','poi']
        # initial_data.bowl_status = 0
        # initial_data.setplace = ''
        # initial_data.breakfast_order = 'Did you say give me an apple, milk and a mouse?'
        # initial_data.breakfast_timeout = (len(initial_data.breakfast_order)+1)/8
        sm.set_initial_state(['AlarmMode'],initial_data)


    return sm	


if __name__ == '__main__':

    rospy.init_node('wakemeup')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('waketest', sm, '/SM_wakemeup')
    sis.start()
    outcome = sm.execute()
    sis.stop()

    