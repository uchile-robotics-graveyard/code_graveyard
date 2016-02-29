#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from bender_msgs.msg import Emotion
from bender_srvs.srv import AskQuestion
from bender_srvs.srv import AskQuestionRequest
from bender_srvs.srv import AskQuestionResponse

from bender_srvs.srv import synthesize
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import SearchPerson
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_srvs.srv import stringReturn

#  - - - - macros - - - -
from bender_macros.vision import WaitOpenDoor
from bender_macros.nav import RoomEntering
from bender_macros.nav import GoToPlace
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.report_generator import GenerateReport
from bender_macros import BringObject
from bender_macros import SeekEmergency
from bender_macros import LookForSomeFace


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class foo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.sleep(0.5)
        
        return 'succeeded'

class PrepareForLook(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):

        # turn on camera right eye
        img_turn_on_client = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_on', Empty)
        img_turn_on_client.wait_for_service()
        img_turn_on_client()

        return 'succeeded'

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

class AnounceWalkout(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
        
    def execute(self, userdata):

        self.talk_client.wait_for_service()
        
        # anounce retreat
        text = 'i will wait for someone to come to help you'
        self.talk_client(text)
        rospy.sleep(2)

        # set normal face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "serious"
        self.face_pub.publish(emotion)

        return 'succeeded'

class ApologizeForBring(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['bring_result']
        )
    
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):

        self.talk_client.wait_for_service()

        if userdata.bring_result == 'not_found':
            # evaluate not_found: sad + apologize

            # set sad3
            emotion = Emotion()
            emotion.Order = "changeFace"
            emotion.Action = "sad3"
            self.face_pub.publish(emotion)

            # apologize
            text = 'i am very sorry human, i was not able to find yor request'
            self.talk_client(text)
            rospy.sleep(5.5)

            # set sad1
            emotion = Emotion()
            emotion.Order = "changeFace"
            emotion.Action = "sad1"
            self.face_pub.publish(emotion)

            return 'succeeded'

        elif userdata.bring_result == 'not_grabbed':
            # evaluate not_grabbed: ashamed + apologize

            # set ashamed
            emotion = Emotion()
            emotion.Order = "changeFace"
            emotion.Action = "ashamed"
            self.face_pub.publish(emotion)

            # apologize
            text = 'i am very ashamed human, i was not able to grasp the object'
            self.talk_client(text)
            rospy.sleep(5.5)

            # set sad1
            emotion = Emotion()
            emotion.Order = "changeFace"
            emotion.Action = "sad1"
            self.face_pub.publish(emotion)

            return 'succeeded'
        
        return 'aborted'

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
class AskForObject(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted'],
                output_keys = ['object']
        )
        
    def execute(self, userdata):
    
        ask_client = rospy.ServiceProxy('/bender/speech/ask_question/askConfirm', AskQuestion)
        ask_req = AskQuestionRequest()
        
        '''
        Now:
         - Do you need water?
            - 'yes'  =>  return 'water'
            - 'no'   =>  next object
         
         - So you need the first aid kit, right?
            - 'yes'  =>  return 'first-aid-kit'
            - 'no'   =>  'aborted'  
          
        Ideas:
        - Do you need something?
        - . . .
        
        '''
        
        ask_req.question = "Do you need water?"
        ask_req.dictionary = "confirmation"
        ask_req.hints = []
        ask_req.qx_sleep = 1.5
        
        while True:
            
            ask_client.wait_for_service()
            ask_res = ask_client(ask_req)
        
            if ask_res.answer == 'yes':
                
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                # TODO: ARREGLAR ESTO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                #userdata.object = 'water'
                userdata.object = 'coca'
                return 'succeeded'
            
            elif ask_res.answer == 'no':
                break
            
        ask_req.question = "So you need the first aid kit, right?"
        ask_req.qx_sleep = 2.0
        
        while True:
            
            ask_client.wait_for_service()
            ask_res = ask_client(ask_req)
        
            if ask_res.answer == 'yes':
                
                userdata.object = 'coca'
                #userdata.object = 'first-aid-kit'
                return 'succeeded'
            
            elif ask_res.answer == 'no':
                break
        
        return 'aborted'

class CheckStatus(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded','aborted','preempted','person_ok','person_in_danger'],
            io_keys=['images','image_captions']
        )

        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
        
    def execute(self, userdata):
        
        ask_client = rospy.ServiceProxy('/bender/speech/ask_question/askConfirm', AskQuestion)
        ask_req = AskQuestionRequest()

        ask_req.question = "Are you ok?"
        ask_req.dictionary = "confirmation"
        ask_req.max_attempts = 3
        ask_req.hints = []
        ask_req.qx_sleep = 1.0
        
        while True:
            
            ask_client.wait_for_service()
            ask_res = ask_client(ask_req)
        
            if ask_res.answer == 'yes':
                
                # Put happy face
                emotion = Emotion()
                emotion.Order = "changeFace"
                emotion.Action = "happy1"
                self.face_pub.publish(emotion)

                # erase false-positive images
                userdata.images = []
                userdata.image_captions = []
                    
                return 'person_ok'
            
            elif ask_res.answer == 'no':
                
                # Put happy face
                emotion = Emotion()
                emotion.Order = "changeFace"
                emotion.Action = "sad2"
                self.face_pub.publish(emotion)

                # aprovechar de sacar ms fotos (para el reporte)
                return 'person_in_danger'
         
        return 'aborted'

class DeliverObject(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],
            input_keys=['grasp_arm']
        )
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        
    def execute(self, userdata):

        # anounce
        text = 'human, catch the object, please'
        self.talk_client(text)
        rospy.sleep(3)

        # soltar el objeto
        # TODO

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

def getMachine():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    ''' Machine variables:
     - map_name            --->   semantic map filename
     - emergency_room      --->   emergency room key on semantic map
     - friend_lookup_place --->   where to stand waiting for the friend
     
     - person_pose         --->   found person     (geometry_msgs/PoseStamped)
     - requested_object    --->   person request   (std_msgs/String)
     - report_img_captions --->   list of captions (std_msgs/String[])
     - report_imgs         --->   list of images   (sensor_msgs/Image[])
    '''
    
    sm.userdata.emergency_room = 'es_emergency_room_look_up_place'
    sm.userdata.friend_lookup_place = 'es_friend_lookup_place'
    sm.userdata.map_name = 'amtc.sem_map'
    sm.userdata.report_imgs = []
    sm.userdata.report_img_captions = []
    
    sm.userdata.person_pose = PoseStamped()
    sm.userdata.friend_pose = PoseStamped()
    sm.userdata.requested_object = 'coca'
    
    # save report
    sm.userdata.device_name = 'KINGSTON'
    
    # talk to friend state
    sm.userdata.ttf_text = 'Please, follow me'
    sm.userdata.ttf_timeout = 2
    
    # Sub Machines - States
    go_to_place_sm = GoToPlace.getInstance()
    approach_to_pose_sm =  ApproachToPoseStamped.getInstance()
    talker = Talk()
    
    # Fill Machine
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                transitions = {'succeeded':'WAIT_FOR_OPEN_DOOR'}
        )
        smach.StateMachine.add('WAIT_FOR_OPEN_DOOR',WaitOpenDoor.getInstance(),
                transitions = {'succeeded':'ROOM_ENTERING'}
        )
        smach.StateMachine.add('ROOM_ENTERING',RoomEntering.Entrance(),
                transitions = {'succeeded':'GO_TO_PERSON_ROOM'}
        )
        smach.StateMachine.add('GO_TO_PERSON_ROOM', go_to_place_sm,
                transitions = {'succeeded':'SEEK_EMERGENCY'},
                remapping = {
                     'place_name':'emergency_room',
                     'map_name':'map_name'}
        )
        smach.StateMachine.add('SEEK_EMERGENCY',SeekEmergency.getInstance(),
                transitions = {'succeeded':'APPROACH_TO_PERSON',
                               'timeout':'CHECK_STATUS'},
                remapping = {'person_pose':'person_pose',
                             'images':'report_imgs',
                             'image_captions':'report_img_captions'}
        )
        smach.StateMachine.add('APPROACH_TO_PERSON', approach_to_pose_sm,
                transitions = { 'succeeded':'CHECK_STATUS'},
                remapping = {'goal_pose':'person_pose'}
        )
        smach.StateMachine.add('CHECK_STATUS', CheckStatus(),
                transitions = {'person_ok':'SEEK_EMERGENCY',
                               'person_in_danger':'ASK_FOR_OBJECT'},
                remapping = {'images':'report_imgs',
                             'image_captions':'report_img_captions'}
        )
        smach.StateMachine.add('ASK_FOR_OBJECT',AskForObject(),
                transitions = {'succeeded':'GENERATE_REPORT',
                               'aborted':'ASK_FOR_OBJECT'},
                remapping = {'object':'requested_object'}
        )
        # NOTA: aunque no se grabe el reporte, se continua adelante
        smach.StateMachine.add('GENERATE_REPORT',GenerateReport.getInstance(),
                transitions = {'succeeded':'BRING_OBJECT',
                               'device_not_found':'BRING_OBJECT',
                               'aborted':'BRING_OBJECT'},
                remapping = {'person_pose':'person_pose',
                             'report_imgs':'report_imgs',
                             'report_img_captions':'report_img_captions',
                             'device_name':'device_name'}
        )
        smach.StateMachine.add('BRING_OBJECT',BringObject.getInstance(),
                transitions = {'succeeded':'DELIVER_OBJECT',
                               'not_found':'APOLOGIZE_FOR_BRING',
                               'not_grabbed':'APOLOGIZE_FOR_BRING'},
                remapping = {'object_name':'requested_object',
                             'grasp_arm':'grasp_arm',
                             'result':'bring_result'}
        )
        smach.StateMachine.add('DELIVER_OBJECT',DeliverObject(),
                transitions = {'succeeded':'ANOUNCE_WALKOUT'},
                remapping = {'grasp_arm':'grasp_arm'}
        )
        smach.StateMachine.add('APOLOGIZE_FOR_BRING',ApologizeForBring(),
                transitions = {'succeeded':'ANOUNCE_WALKOUT'},
                remapping = {'bring_result':'bring_result'}
        )
        smach.StateMachine.add('ANOUNCE_WALKOUT',AnounceWalkout(),
                transitions = {'succeeded':'GO_TO_ENTRY'}
        )
        smach.StateMachine.add('GO_TO_ENTRY', go_to_place_sm,
                transitions = {'succeeded':'PREPARE_FOR_LOOK'},
                remapping = {
                     'place_name':'friend_lookup_place',
                     'map_name':'map_name'}
        )
        smach.StateMachine.add('PREPARE_FOR_LOOK',PrepareForLook(),
                transitions = {'succeeded':'LOOK_FOR_FRIEND'}
        )
        smach.StateMachine.add('LOOK_FOR_FRIEND',LookForSomeFace.getInstance(),
                transitions = {'succeeded':'APPROACH_TO_FRIEND',
                               'aborted':'LOOK_FOR_FRIEND'},
                remapping = {'sm_face_pose':'friend_pose'}
        )
        smach.StateMachine.add('APPROACH_TO_FRIEND', approach_to_pose_sm,
           transitions = {
                  'succeeded':'LOOK_FRIEND',
                  'aborted':'LOOK_FOR_FRIEND'
           },
           remapping = {'goal_pose':'friend_pose'}
        )
        smach.StateMachine.add('LOOK_FRIEND', LookToPoseStamped.getInstance(),
           transitions = {
                  'succeeded':'DETECT_FRIEND_FACE',
                  'aborted':'LOOK_FOR_FRIEND'
           },
           remapping = {'goal_pose':'friend_pose'}
        )
        
        def detect_face_response_cb(userdata, response):
            rospy.loginfo('number of detected faces: ' + str(response.n_faces))
            if response.n_faces > 0:
                return 'succeeded'
            else:
                return 'aborted'

        smach.StateMachine.add('DETECT_FRIEND_FACE',
            smach_ros.ServiceState('/bender/vision/face_detector/detect_face',
                FaceInfo,
                response_cb = detect_face_response_cb
            ),
            transitions = {
                'succeeded':'TALK_TO_FRIEND',
                'aborted':'LOOK_FOR_FRIEND'
            }
        )
        
        smach.StateMachine.add('TALK_TO_FRIEND',talker,
                transitions = {'succeeded':'GUIDE_TO_PERSON'},
                remapping = {'text':'ttf_text',
                             'timeout':'ttf_timeout'}
        )
        
        smach.StateMachine.add('GUIDE_TO_PERSON', approach_to_pose_sm,
           transitions = { 'succeeded':'succeeded'},
           remapping = {'goal_pose':'person_pose'}
        )
               
        # dummy userdata
        #person_pose = PoseStamped()
        #person_pose.header.frame_id = '/map'
        #person_pose.header.stamp = rospy.Time.now()
        #person_pose.pose.position.x = -2.13100624084473
        #person_pose.pose.position.y = -4.90687799453735
        #person_pose.pose.orientation.x = 0.77004861440132
        #person_pose.pose.orientation.w = 1.0
        
        ud = smach.UserData()
        #ud.person_pose = person_pose
        
        sm.set_initial_state(['SETUP'], ud)
        
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('emergency_situation')

    sm = getMachine()

    # introspection server
    sis = smach_ros.IntrospectionServer('emergency_situation', sm, '/EMERGENCY_SITUATION_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
