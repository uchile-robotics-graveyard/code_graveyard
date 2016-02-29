#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from bender_msgs.msg import Emotion
from bender_srvs.srv import AskQuestion
from bender_srvs.srv import AskQuestionRequest
from bender_srvs.srv import AskQuestionResponse
from bender_srvs.srv import stringReturn
from bender_srvs.srv import synthesize
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import SearchPerson
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_srvs.srv import stringReturn
from bender_srvs.srv import Transformer
from bender_srvs.srv import TransformerRequest
from bender_srvs.srv import TransformerResponse



#  - - - - macros - - - -
from bender_macros.vision import WaitOpenDoor
from bender_macros import LookForSomeFace
from bender_macros.nav import GoToPlace
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.arm import DropBasket
from bender_macros.arm import GraspObject
from bender_macros import GoForObject
from bender_macros.vision import DetectObjects

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n 
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):
        
        self.talk_client.wait_for_service()

        # Put happy face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy1"
        self.face_pub.publish(emotion)

        # quizas hacer un wait for service de todo?

        # anounce ready
        text = 'i am ready to start'
        self.talk_client(text)
        rospy.sleep(2)

        return 'succeeded'

class GetValidObjects(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','no_match','aborted','preempted'],
            input_keys = ['required_objects','detection_ids','detection_positions','detection_types','detection_status'],
            output_keys = ['valid_ids','valid_positions','valid_types','valid_status']
        )
        
    def execute(self, userdata):

        available_object_ids = []
        available_object_positions = []
        available_object_types = []
        
        # look for object in detections array
        for i in range(len(userdata.detection_ids)):

            if userdata.detection_ids[i] in userdata.required_objects and userdata.detection_status[i] == "in_range":

                available_object_ids = available_object_ids.append(userdata.detection_ids[i])
                available_object_types = available_object_types.append(userdata.detection_types[i])
                available_object_positions = available_object_positions.append(userdata.detection_positions[i])

        if len(available_object_ids) == 0:
            return 'no_match'
    
        return 'succeeded'

class SelectValidObject(smach.State):

    def __init__(self):
    
        smach.State.__init__(
            self,
            outcomes = ['continue','succeeded','aborted','preempted'],
            input_keys = ['valid_ids', 'valid_positions', 'valid_types', 'valid_status', 'required_objects'],
            output_keys = ['selected_id', 'selected_position', 'selected_type', 'selected_status']
        )

        self.tried_objects = []
    
    def execute(self, userdata):
        
        if len(self.tried_objects) == len(required_objects):
            return 'continue'

        for i in range(len(userdata.valid_ids)):

            if userdata.valid_ids[i] not in self.tried_objects:

                userdata.selected_id =  userdata.valid_ids[i]
                userdata.selected_position =  userdata.valid_positions[i]
                userdata.selected_type =  userdata.valid_types[i]
                userdata.selected_status =  userdata.valid_status[i]

                self.tried_objects = self.tried_objects.append(selected_id)

                return 'succeeded'

class ApologizeForGrasp(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted']
        )
    
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):
        # evaluate not_grabbed: ashamed + apologize

        self.talk_client.wait_for_service()

        # set ashamed
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "ashamed"
        self.face_pub.publish(emotion)

        # apologize
        text = 'i am very ashamed, i was not able to grasp the object'
        self.talk_client(text)
        rospy.sleep(4)

        # set sad1
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "sad1"
        self.face_pub.publish(emotion)

        return 'succeeded'

class ReactToGrasp(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['selected_id'],
            io_keys=['grab_list']
        )
               
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):

        self.talk_client.wait_for_service()

        # set ashamed
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy1"
        self.face_pub.publish(emotion)

        userdata.grab_list = userdata.grab_list.append(userdata.selected_id)

        return 'succeeded'
 
class AskInformation(smach.State):

    def __init__(self):
	self.contperson=0
        smach.State.__init__(
                self,
                outcomes = ['continue','succeeded','aborted','preempted'],
                input_keys = ['pose','person_pose'],  
                output_keys = ['object','name','pose']
        )
        
    def execute(self, userdata):
    

        ask_name = rospy.ServiceProxy('/bender/speech/interaction/request_name', stringReturn)
        ask_name.wait_for_service()
        name_res = ask_name()
        
        face = rospy.ServiceProxy('/add_face', ID)
        face.wait_for_service()
        face_res = face(self.contperson)
        while face_res.ID==0:
		print face_res.ID
		face_res = face(self.contperson)
	    
        ask_obj = rospy.ServiceProxy('/bender/speech/interaction/request_drink', stringReturn)
        ask_obj.wait_for_service()
        obj_res = ask_obj()
        
        userdata.pose[self.contperson]=userdata.person_pose
        userdata.name[self.contperson]=name_res
        userdata.object[self.contperson]=obj_res

        rospy.loginfo(userdata.name[self.contperson])
        rospy.loginfo(userdata.object[self.contperson])
        self.contperson+=1
	   
        if self.contperson==3:
		print userdata.name[0]+userdata.object[0]
		print userdata.name[1]+userdata.object[1]
		print userdata.name[2]+userdata.object[2]
	       	return 'succeeded'
	    
	return 'continue'

class DeliverObject(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['deliver_name', 'deliver_object','deliver_object_location']
        )
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        
    def execute(self, userdata):

        if userdata.deliver_object_location == "arm":

            # anounce
            text = userdata.deliver_name + ", please, take the " + userdata.deliver_object + " from my arm"
            self.talk_client(text)
            rospy.sleep(5)

            return 'succeeded'

        # anounce
        text = userdata.deliver_name + ", please, take the " + userdata.deliver_object + " from the " + userdata.deliver_object_location + " of my basket"
        self.talk_client(text)
        rospy.sleep(5)

        return 'succeeded'

class Talk(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted'],
                input_keys = ['text','timeout']
        )
        
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        
    def execute(self, userdata):
        
        self.talk_client.wait_for_service()
        self.talk_client(userdata.text)
        rospy.sleep(userdata.timeout)
                
        return 'succeeded'

class searchWaving(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted'],
                input_keys = ['text','timeout'],
                output_keys = ['person_pose']
        )
        self.wave_client = rospy.ServiceProxy('/bender/macros/search_waving',SearchPerson)
        self.trans_client = rospy.ServiceProxy('/bender/tf/simple_pose_transformer/transform',Transformer)

    def execute(self, userdata):

        self.wave_client.wait_for_service()
        res=self.wave_client()

        while res.found==False:
            res=self.wave_client()

            rospy.sleep(1)

        # transf
        res.person.header.stamp = rospy.Time.now()

        tf_srv=TransformerRequest()
        tf_srv.frame_out = "/map"
        tf_srv.pose_in = res.person

        tf_res = self.trans_client(tf_srv)

        userdata.person_pose = tf_res.pose_out

        return 'succeeded'
        
class SelectBasket(smach.State):

    def __init__(self):
    
        smach.State.__init__(
            self,
            outcomes = ['basket_full','succeeded','aborted','preempted'],
            input_keys = ['selected_arm','selected_id'],
            output_keys = ['basket_side'],
            io_keys = ['basket_object_map']
        )

        self.occupied_locations = []
    
    def execute(self, userdata):
        
        if len(self.occupied_locations) == 2:
            userdata.basket_object_map =  userdata.basket_object_map.append((userdata.selected_id,"arm"))
            return 'basket_full'

        if (userdata.selected_arm == "/left_arm") and ("left" not in self.occupied_locations):
            userdata.basket_object_map =  userdata.basket_object_map.append((userdata.selected_id,"left"))
            userdata.basket_side = "left"

        elif (userdata.selected_arm == "/right_arm") and ("right" not in self.occupied_locations):
            userdata.basket_object_map =  userdata.basket_object_map.append((userdata.selected_id,"right"))
            userdata.basket_side = "right"

        elif "left" not in self.occupied_locations:
            userdata.basket_object_map =  userdata.basket_object_map.append((userdata.selected_id,"left"))
            userdata.basket_side = "left"

        elif "right" not in self.occupied_locations:
            userdata.basket_object_map =  userdata.basket_object_map.append((userdata.selected_id,"right"))
            userdata.basket_side = "right"

        return 'succeeded' 

class PrepareDeliver(smach.State):

    def __init__(self):
    
        smach.State.__init__(
            self,
            outcomes = ['continue','succeeded','aborted','preempted'],
            input_keys = ['basket_object_map', 'required_objects', 'requested_name', 'requested_pose'],
            output_keys = ['deliver_object','deliver_pose', 'deliver_name', 'deliver_object_location']
        )

        self.delivered_objects = []
    
    def execute(self, userdata):
        
        if len(self.delivered_objects) == len(self.basket_object_map):
            return 'continue'
        
        for i in range(len(self.basket_object_map)):

            if (userdata.basket_object_map[i])[1] not in self.delivered_objects:

                for j in range(len(userdata.required_objects)):
                    if userdata.required_objects[j] == (userdata.basket_object_map[i])[1]:
                        userdata.deliver_object = userdata.required_objects[j]
                        userdata.deliver_name = userdata.requested_name[j]
                        userdata.deliver_pose = userdata.requested_pose[j]
                        userdata.deliver_object_location = (userdata.basket_object_map[i])[0]
                        
                        return 'succeeded'

        return 'aborted'

def getMachine():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    ''' Machine variables:
     - map_name         --->   semantic map filename
     - cocktail_room   --->   room key on semantic map
     - obj_place      --->   entry key on semantic map
     
     - person_pose         --->   found person     (geometry_msgs/PoseStamped)
     - required_objects    --->   person request   (std_msgs/String)
     
    '''
    sm.userdata.exit_place = 'door_left_post_out'
    sm.userdata.cocktail_room = 'living_room'
    sm.userdata.obj_place = 'bar'
    sm.userdata.map_name = 'map.sem_map'
    
    sm.userdata.person_pose = PoseStamped()
    sm.userdata.requested_pose = ['','','']
    sm.userdata.requested_name = ['','','']
    sm.userdata.required_objects = ['','','']
    sm.userdata.basket_object_map = []
    sm.userdata.basket_side = -1

    # sm.userdata.required_objects
    # sm.userdata.detection_ids
    # sm.userdata.detection_positions
    # sm.userdata.detection_types
    # sm.userdata.detection_status
    # sm.userdata.valid_ids
    # sm.userdata.valid_positions
    # sm.userdata.valid_types
    # sm.userdata.valid_status
    # sm.userdata.grab_list


    # Sub Machines - States
    go_to_place_sm = GoToPlace.getInstance()
    approach_to_pose_sm =  ApproachToPoseStamped.getInstance()

    talker = Talk()
    drop_basket = DropBasket.getInstance()
        
    # Fill Machine
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                transitions = {'succeeded':'WAIT_FOR_OPEN_DOOR'}
        )
        smach.StateMachine.add('WAIT_FOR_OPEN_DOOR',WaitOpenDoor.getInstance(),
                transitions = {'succeeded':'GO_TO_PERSON_ROOM'}
        )

        smach.StateMachine.add('GO_TO_PERSON_ROOM', go_to_place_sm,
                transitions = {'succeeded':'LOOK_FOR_WAVING'},
                remapping = {'place_name':'cocktail_room',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('GO_TO_PERSON_ROOM', go_to_place_sm,
                transitions = {'succeeded':'LOOK_FOR_WAVING'},
                remapping = {
                     'place_name':'cocktail_room',
                     'map_name':'map_name'}
        )
        smach.StateMachine.add('LOOK_FOR_WAVING',searchWaving(),
                transitions = {'succeeded':'APPROACH_TO_PERSON'},
                remapping = {'person_pose':'person_pose'}
        )
        smach.StateMachine.add('APPROACH_TO_PERSON', approach_to_pose_sm,
                transitions = { 'succeeded':'AskInformation'},
                remapping = {'goal_pose':'person_pose'}
        )
        smach.StateMachine.add('AskInformation',AskInformation(),
                transitions = {'succeeded':'succeeded',#GO_TO_KITCHEN
                             'continue':'GO_TO_PERSON_ROOM'},
                remapping = {'person_pose':'person_pose',
                            'object':'requested_object',
                             'name':'requested_name',
                             'pose':'requested_pose'}
        )
        smach.StateMachine.add('GO_TO_BAR', go_to_place_sm,
                transitions = {'succeeded':'DETECT_OBJECTS'},
                remapping = {'place_name':'obj_place',
                             'map_name':'map_name'}
        )
        # -- - TODO: no_detections --> moverse hacia al lado -- -
        smach.StateMachine.add('DETECT_OBJECTS',DetectObjects.getInstance(),
            transitions = {'succeeded':'GET_VALID_OBJECTS',
                           'no_detections':'DETECT_OBJECTS'},
            remapping = {'object_positions':'detection_positions',
                         'object_ids':'detection_ids',
                         'object_types':'detection_types',
                         'object_status':'detection_status'}
        )
        smach.StateMachine.add('GET_VALID_OBJECTS',GetValidObjects(),
            transitions = {'succeeded':'SELECT_VALID_OBJECT',
                           'no_match':'DETECT_OBJECTS'},
            remapping = {'required_objects':'required_objects',
                         'detection_ids':'detection_ids',
                         'detection_positions':'detection_positions',
                         'detection_types':'detection_types',
                         'detection_status':'detection_status',
                         'valid_ids':'valid_ids',
                         'valid_positions':'valid_positions',
                         'valid_types':'valid_types',
                         'valid_status':'valid_status'
            }
        )
        # AHORA, SI NO PUDO TOMAR TODOS LOS OBJETOS, ENTONCES, RETORNARA A ENTREGAR LOS NECESARIOS
        smach.StateMachine.add('SELECT_VALID_OBJECT', SelectValidObject(),
                transitions = {
                    'succeeded':'GRASP_OBJECT',
                    'continue':'PREPARE_DELIVER'},
                remapping = {
                    'required_objects':'required_objects',
                    'valid_ids':'valid_ids',
                    'valid_positions':'valid_positions',
                    'valid_types':'valid_types',
                    'valid_status':'valid_status',
                    'selected_id':'selected_id',
                    'selected_position':'selected_position',
                    'selected_type':'selected_type',
                    'selected_status':'selected_status',
                }
        )
        smach.StateMachine.add('GRASP_OBJECT',GraspObject.getInstance(),
                transitions = {'succeeded':'SELECT_BASKET',
                           'notGrabbed':'APOLOGIZE_FOR_GRASP'},
                remapping = {'position':'selected_position',
                         'selected_arm':'grasp_arm'}
        )
        smach.StateMachine.add('APOLOGIZE_FOR_GRASP',ApologizeForGrasp(),
                transitions = {'succeeded':'SELECT_VALID_OBJECT'},
                remapping = {'grab_list':'grab_list'}
        )
        smach.StateMachine.add('REACT_TO_GRASP',ReactToGrasp(),
                transitions = {'succeeded':'SELECT_BASKET'},
                remapping = {'grab_list':'grab_list',
                             'selected_id':'selected_id'}
        )
        smach.StateMachine.add('SELECT_BASKET',SelectBasket(),
                transitions = {'succeeded':'DROP_BASKET',
                               'basket_full':'PREPARE_DELIVER'},
                remapping = {'basket_side':'basket_side',
                             'selected_id':'selected_id',
                             'selected_arm':'selected_arm',
                             'basket_object_map':'basket_object_map'}
        )
        smach.StateMachine.add('DROP_BASKET',DropBasket.getInstance(),
                transitions = {'succeeded':'SELECT_VALID_OBJECT'},
                remapping = {'basket_side':'basket_side',
                             'selected_arm':'selected_arm'}
        )
        smach.StateMachine.add('PREPARE_DELIVER',PrepareDeliver(),
                transitions = {'succeeded':'APPROACH_TO_PERSON_DELIVER',
                               'continue':'GO_TO_EXIT'},
                remapping = {'basket_object_map':'basket_object_map',
                             'required_objects':'required_objects',
                             'requested_name':'requested_name',
                             'requested_pose':'requested_pose',
                             'deliver_pose':'deliver_pose',
                             'deliver_name':'deliver_name',
                             'deliver_object':'deliver_object',
                             'deliver_object_location':'deliver_object_location'}
        )
        smach.StateMachine.add('APPROACH_TO_PERSON_DELIVER', approach_to_pose_sm,
                transitions = {'succeeded':'DELIVER_OBJECT'},
                remapping = {'goal_pose':'deliver_pose'}
        )
        smach.StateMachine.add('DELIVER_OBJECT',DeliverObject(),
                transitions = {'succeeded':'PREPARE_DELIVER'},
                remapping = {'deliver_name':'deliver_name',
                             'deliver_object':'deliver_object',
                             'deliver_object_location':'deliver_object_location'}
        )
        smach.StateMachine.add('GO_TO_EXIT', go_to_place_sm,
                transitions = {'succeeded':'succeeded'},
                remapping = {'place_name':'exit_place',
                             'map_name':'map_name'}
        )
        
        ud = smach.UserData()
        #ud.person_pose = person_pose

        sm.set_initial_state(['AskInformation'], ud)
        
        
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('cocktail_party')

    sm = getMachine()

    # introspection server
    sis = smach_ros.IntrospectionServer('cocktail_party', sm, '/COCKTAIL_PARTY_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
