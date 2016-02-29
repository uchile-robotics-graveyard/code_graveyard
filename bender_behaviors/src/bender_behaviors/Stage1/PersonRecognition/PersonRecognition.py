#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import math
import smach
import smach_ros
import cv2

from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

#  - - - - macros - - - -
from bender_macros.head import MoveAsus
from bender_macros.head import FaceOrder
from bender_macros.nav import ApproachAndLook
from bender_macros.nav import RotateRobot
from bender_macros.nav import GoToPlace
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.speech import Talk
from bender_macros.speech import TalkState
from bender_macros.speech import WaitCommand
from bender_macros.vision import EnrollFace_RGBD
from bender_macros.vision import DetectGender
from bender_macros.vision import DetectStateCrowd
from bender_macros.vision import DetectGenderCrowd
from bender_macros.vision import FindFace
from bender_macros.vision import EmotionRecognition
from bender_macros.vision import SearchCrowd

from bender_behaviors.Stage1.PersonRecognition import TrainingPerson
from bender_behaviors.Stage1.PersonRecognition import SearchOperator

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf import transformations

from cv2 import imshow, waitKey
from bender_core import benpy

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class Setup(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                    io_keys=['init_time'])
        
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
    
    def execute(self, userdata):
        
        userdata.init_time = rospy.Time.now()
        
        # Put happy face and center neck
        FaceOrder.ChangeFace("happy1")
        FaceOrder.ChangeFace(0)
        
        # anounce ready
        Talk.getInstance('I am ready to start')
        
        return 'succeeded'

class HandleNotFoundTimed(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['continue-now','can-retry'],
                    input_keys=['init_time'])

        # 6 min
        self.test_duration = rospy.Duration(6*60)
        
        # 2 min
        self.remaining_th = rospy.Duration(2*60)

    def execute(self, userdata):
        
        curr_time = rospy.Time.now()
        elapsed_time = curr_time - userdata.init_time
        remaining_time = self.test_duration - elapsed_time
        if remaining_time > self.remaining_th:
            return 'can-retry'
        
        return 'continue-now'

class SearchOperatorConfirm(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                outcomes=['not_found','succeeded','aborted','preempted'])
        
         # ROS interfaces
        self.face_detection_client   = benpy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
        self.face_recognition_client = benpy.ServiceProxy('/bender/vision/face_recognizer/recognize', FaceRecognition)
        self.transform_client = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)
        self.rgb_topic   = '/bender/sensors/rgbd_head/rgb/image_raw'
        self.depth_topic = '/bender/sensors/rgbd_head/depth/image_raw'
        self.rgb_subs   = None
        self.depth_subs = None
        
        # ROS data   
        self.rgb_image   = None
        self.depth_image = None
        
        # ROS -> opencv data
        self.bridge = CvBridge()
        self.cv_rgb_image   = []
        self.cv_depth_image = []
        
        # control
        self.max_attempts = 5
        self.max_score_th = 100
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False

        # start on "pause" state
        self.pause_dataflow()
        
    def init_dataflow(self):
        
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if not self.rgb_subs:
            self.rgb_subs   = rospy.Subscriber(self.rgb_topic, sensor_msgs.msg.Image, self.rgb_cb)
            self.depth_subs = rospy.Subscriber(self.depth_topic, sensor_msgs.msg.Image, self.depth_cb)
    
    def pause_dataflow(self):
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if self.rgb_subs:
            self.rgb_subs.unregister()
            self.depth_subs.unregister()
        self.rgb_subs   = None
        self.depth_subs = None
    
    def reset_images(self):
        self.got_images = False
        self.got_images_to_cv2 = False
    
    def rgb_cb(self,msg):
        self.rgb_image = msg
        self.got_rgb_img = True
    
    def depth_cb(self,msg):
        self.depth_image = msg
        self.got_depth_img = True
    
    def get_rgbd_data(self):
        
        self.init_dataflow()
        if not self.got_images:
            
            rospy.sleep(0.5)
            
            while not (self.got_rgb_img and self.got_depth_img):
                rospy.sleep(0.1)
                
            self.got_images = True

        if not self.got_images_to_cv2:
            try:
                self.cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
                self.cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, "16UC1")
            except rospy.ServiceException as e:
                rospy.logerr("ConfirmOperator: failed to convert ROS image to OpenCV image. why?: " + str(e))
                self.pause_dataflow()
                return False
            self.got_images_to_cv2 = True
        
        self.pause_dataflow()
        return True

    def execute(self, userdata):
        
        rospy.loginfo('Executing state: SEARCH_OPERATOR')
        Talk.getInstance("Hi, i am looking for a friend, ... please look at my hat", 6)
        
        # n attempts to recognize the operator
        attempt_no = 0
        while attempt_no < self.max_attempts:
        
            if not self.get_rgbd_data():
                rospy.sleep(0.2)
                attempt_no += 1
                continue
            
            req = FaceRecognitionRequest()
            req.use_image = True
            req.image     = self.rgb_image
            
            try:
                resp = self.face_recognition(req)
                
                if resp.distance < self.max_score_th:
                    FaceOrder.ChangeFace("happy2")
                    return 'succeeded'

            except rospy.ServiceException as e:
                rospy.logwarn("ConfirmOperator: Ups. Failed to call the face recognize server. why?: " + str(e))
            attempt_no += 1
        
        Talk.getInstance("i am sorry human, i am looking for someone else")
        FaceOrder.ChangeFace("sad2")

        return 'not_found'

class TalkInfoOperator(smach.State):

    def __init__(self):
        smach.State.__init__(
                self, outcomes = ['succeeded','aborted','preempted'],
                     input_keys=['operator_gender','operator_pose','operator_emotion'])
           
    def execute(self, userdata):
        rospy.loginfo('-Operator Information :')
        rospy.loginfo('Emotion: ' + str(userdata.operator_emotion))
        rospy.loginfo('Gender : ' + str(userdata.operator_gender))
        rospy.loginfo('Pose   : ' + str(userdata.operator_pose))
        #rospy.loginfo('Info in crowd: ' + str(userdata.person_infcrowd))

        sp  = "Hi operator, you are the "
        if userdata.operator_emotion:
            sp += str(userdata.operator_emotion) + " "
        sp += str(userdata.operator_gender) + " human."
        
        Talk.getInstance(sp)
       #TODO TODO sp = "In the crowd you are the " + str(userdata.person_infcrowd) + " who is " + str(userdata.person_state)
        
        return 'succeeded'

class TalkInfoCrowd(smach.State):

    def __init__(self):
        smach.State.__init__(
                self, outcomes = ['succeeded','aborted','preempted'],
                    input_keys=['n_women','n_men'])
        
    def execute(self, userdata):

        rospy.loginfo('-Crowd Information :')
        rospy.loginfo('Male : ' + str(userdata.n_men))
        rospy.loginfo('Women: ' + str(userdata.n_women))
        rospy.loginfo('Total: ' + str(userdata.n_men + userdata.n_women))

        text  = "In the crowd i found " + str(userdata.n_men+userdata.n_women) + " people, from which "
        text += str(userdata.n_women) + " are women and " + str(userdata.n_men) + " are men"
        Talk.getInstance(text)
     
        return 'succeeded'

class HandleCrowdsGoals(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted','finished'],
            input_keys=['crowd_go_poses','crowd_centers','crowd_indexes',
                        'crowd_images', 'crowd_ROIs', 'crowd_ROIs_boxes', 'crowd_person_poses'],
            output_keys=['next_go_pose','next_look_pose','next_crowd_size_hint',
                     'next_crowd_images', 'next_crowd_ROIs', 'next_crowd_ROIs_boxes', 'next_crowd_person_poses'])
    
        self.current_idx = 0
    
    def execute(self, userdata):

        # available values
        n_poses = min(len(userdata.crowd_go_poses),len(userdata.crowd_centers))
        
        # we have finished
        if self.current_idx >= n_poses:
            #Talk.getInstance("last crowd",1)
            self.current_idx = 0 
            return 'finished'
        
        # load values
        userdata.next_go_pose = userdata.crowd_go_poses[self.current_idx]
        userdata.next_look_pose = userdata.crowd_centers[self.current_idx]
        
        next_crowd_images = []
        next_crowd_ROIs   = []
        next_crowd_ROIs_boxes   = []
        next_crowd_person_poses = []
        
        n_idxs = len(userdata.crowd_indexes)
        for k in range(n_idxs):
        
            next_crowd_images.append(userdata.crowd_images[k])
            next_crowd_ROIs.append(userdata.crowd_ROIs[k])
            next_crowd_ROIs_boxes.append(userdata.crowd_ROIs_boxes[k])
            next_crowd_person_poses.append(userdata.crowd_person_poses[k])
            
        userdata.next_crowd_size_hint    = n_idxs
        userdata.next_crowd_images       = next_crowd_images    
        userdata.next_crowd_ROIs         = next_crowd_ROIs    
        userdata.next_crowd_ROIs_boxes   = next_crowd_ROIs_boxes    
        userdata.next_crowd_person_poses = next_crowd_person_poses    

        print "next go pose"
        print userdata.crowd_go_poses[self.current_idx]
        print "next look pose"
        print userdata.crowd_centers[self.current_idx]

        # prepare for next round
        self.current_idx += 1 
        #Talk.getInstance("I'm going to the next crowd",2)
        return 'succeeded'

class HandleCrowdsData(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'],
            input_keys  = ['operator_pose', 'operator_score', 'n_women_curr', 'n_men_curr'],
            output_keys = ['operator_pose_best', 'operator_score_best','n_women', 'n_men'])
        
        self.current_idx = 0
    
    def execute(self, ud):
        
        # append detections
        ud.n_women += ud.n_women_curr
        ud.n_men   += ud.n_men_curr
        
        # we prefer the last seen operator over the first one, so we are closer!
        if ud.operator_score >= ud.operator_score_best:
            ud.operator_score_best = ud.operator_score
            ud.operator_pose_best  = ud.operator_pose
        
        return 'succeeded'

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    # - - -  machine parameters - - -    
    # -- navigation -- 
    sm.userdata.turn_around_angle = 180  # angle to turn after person enrolling

    # -- crowd analysis --
    sm.userdata.n_clusters = 2
    sm.userdata.angle_operator_search = 20
    
    
    # - - -  machine data  - - -
    # elapsed time
    sm.userdata.init_time = None
    
    # -- operador info --
    sm.userdata.operator_emotion = None
    sm.userdata.operator_gender = "male"
    sm.userdata.operator_pose   = None
    sm.userdata.operator_score  = 1000
    sm.userdata.operator_pose_best  = None
    sm.userdata.operator_score_best = 1000
    sm.userdata.angle_operator_face = 0

    # -- crowd info --
    # crowd analysis
    sm.userdata.crowd_images       = None # 
    sm.userdata.crowd_ROIs_boxes   = None # 
    sm.userdata.crowd_ROIs         = None #
    sm.userdata.crowd_person_poses = None #
    sm.userdata.crowd_indexes      = None #
    
    # go/look poses
    sm.userdata.crowd_go_poses  = None # crowd poses for robot approach
    sm.userdata.crowd_centers   = None # crowd poses to look
    sm.userdata.next_go_pose    = None
    sm.userdata.next_look_pose  = None
    
    # crowd info
    sm.userdata.crowd_states = [] # must be an array!
    sm.userdata.n_women = 0       # must be zero
    sm.userdata.n_men   = 0       # must be zero
    sm.userdata.n_men_curr   = 0
    sm.userdata.n_women_curr = 0
    # -----------------

    #sm.userdata.next_crowd_size_hint = 2
    # Fill Machine
    with sm:

        # -- -- robot setup -- --
        smach.StateMachine.add('SETUP', Setup(),
            transitions = {'succeeded':'TRAINING_PERSON'},
            remapping   = {'init_time':'init_time'}
        )

        # -- -- operator enroll -- --
        smach.StateMachine.add('TRAINING_PERSON', TrainingPerson.getInstance(),
            transitions = {'succeeded':'ANNOUNCE_MEMORIZED',
                           'aborted':'ANNOUNCE_MEMORIZED'},
            remapping = {'operator_gender':'operator_gender',
                         'angle_rgbd':'angle_operator_face'} 
        )
        
        smach.StateMachine.add('ANNOUNCE_MEMORIZED',
            TalkState.getInstance('I am ready operator, ... i will wait here for some time', 4),
            transitions = {'succeeded':'WAIT_STARTING_COMMAND'}
        )
   
        # -- -- crowd lookup -- --
        smach.StateMachine.add('WAIT_STARTING_COMMAND', WaitCommand.getInstance(),
            transitions = {'succeeded':'TURN_AROUND'}
        )
        
        smach.StateMachine.add('TURN_AROUND', RotateRobot.getInstance(),
            transitions = {'succeeded':'ANNOUNCE_SEEK',
                           'aborted':'ANNOUNCE_SEEK'},
            remapping   = {'angle':'turn_around_angle'}
        )
        
        smach.StateMachine.add('ANNOUNCE_SEEK',
            TalkState.getInstance('now, i will look for a crowd', 3),
            transitions = {'succeeded':'SEARCH_CROWD'}
        )
        
        smach.StateMachine.add('SEARCH_CROWD', SearchCrowd.getInstance(),
            transitions = {'succeeded':'HANDLE_CROWDS',
                           'aborted':'SEARCH_CROWD'},
            remapping = {'n_clusters':'n_clusters',
                         'crowd_go_poses':'crowd_go_poses',
                         'crowd_centers':'crowd_centers',
                         'crowd_images':'crowd_images',
                         'crowd_ROIs':'crowd_ROIs',
                         'crowd_ROIs_boxes':'crowd_ROIs_boxes',
                         'crowd_person_poses':'crowd_person_poses',
                         'crowd_indexes':'crowd_indexes'}
        )
        
        # -- -- crowd interaction -- --
        smach.StateMachine.add('HANDLE_CROWDS', HandleCrowdsGoals(),
            transitions = {'succeeded':'APPROACH_TO_CROWD',
                           'finished':'SEARCH_OPERATOR'},
            remapping = {'crowd_go_poses':'crowd_go_poses',
                         'crowd_centers':'crowd_centers',
                         'crowd_images':'crowd_images',
                         'crowd_ROIs':'crowd_ROIs',
                         'crowd_ROIs_boxes':'crowd_ROIs_boxes',
                         'crowd_person_poses':'crowd_person_poses',
                         'next_go_pose':'next_go_pose',
                         'next_look_pose':'next_look_pose',
                         'next_crowd_images':'next_crowd_images',
                         'next_crowd_ROIs':'next_crowd_ROIs',
                         'next_crowd_ROIs_boxes':'next_crowd_ROIs_boxes',
                         'next_crowd_person_poses':'next_crowd_person_poses',
                         'next_crowd_size_hint':'next_crowd_size_hint'
                         }
        )
        
        smach.StateMachine.add('APPROACH_TO_CROWD', ApproachAndLook.getInstance(),
            transitions = {'succeeded':'ANNOUNCE_ANALYZE',
                           'aborted':'APPROACH_TO_CROWD'},
            remapping = {'pose_approach':'next_go_pose',
                         'pose_look':'next_look_pose'}
        )
        
        smach.StateMachine.add('ANNOUNCE_ANALYZE',
            TalkState.getInstance('hi, can you look at me?... i am looking for a friend of mine', 5),
            transitions = {'succeeded':'MOVE_ASUS_ASDF'}
        )

        smach.StateMachine.add('MOVE_ASUS_ASDF', MoveAsus.getReadyMachine(15),
            transitions={'succeeded':'CROWD_GENDER_RECOGNITION'}
        )

        smach.StateMachine.add('CROWD_GENDER_RECOGNITION', DetectGenderCrowd.getInstance(),
            transitions = {'succeeded':'SEARCH_OPERATOR',
                           'aborted':'SEARCH_OPERATOR'},
            remapping = {'crowd_size_hint':'next_crowd_size_hint',
                         'n_women':'n_women_curr',
                         'n_men':'n_men_curr'}
        )
        
        smach.StateMachine.add('SEARCH_OPERATOR', SearchOperator.getInstance(),
            transitions={'succeeded':'HANDLE_CROWDS_DATA',
                         'aborted':'HANDLE_CROWDS'},
            remapping = {'rgbd_angle':'angle_operator_search',
                         'operator_pose' :'operator_pose',
                         'crowd_size_hint':'next_crowd_size_hint',
                         'operator_score':'operator_score'}
        )
                
        smach.StateMachine.add('HANDLE_CROWDS_DATA', HandleCrowdsData(),
            transitions={'succeeded':'HANDLE_CROWDS',
                         'aborted':'HANDLE_CROWDS'},
            remapping = {'operator_pose' :'operator_pose',
                         'operator_score':'operator_score',
                         'n_women':'n_women',
                         'n_men':'n_men',
                         'operator_pose_best':'operator_pose_best',
                         'operator_score_best':'operator_score_best',
                         'n_women_curr':'n_women_curr',
                         'n_men_curr':'n_men_curr'}
        )
        
        # -- -- Operator Interaction -- -- 
        smach.StateMachine.add('APPROACH_TO_OPERATOR', ApproachToPoseStamped.getInstance(), 
            transitions={'succeeded':'MOVE_ASUS_CONFIRM',
                        'aborted':'APPROACH_TO_OPERATOR'},
            remapping = {'goal_pose':'operator_pose_best'}
        )
        
        
        smach.StateMachine.add('MOVE_ASUS_CONFIRM', MoveAsus.getInstance(),
            transitions={'succeeded':'OPERATOR_CONFIRM'},
            remapping={'deg_angle':'angle_operator_face'}
        )
        
        # todo: mover el cuello para verificar
        smach.StateMachine.add('OPERATOR_CONFIRM', SearchOperatorConfirm(),
            transitions={'succeeded':'EMOTION_RECOGNITION',
                         #'not_found':'HANDLE_NOT_FOUND_TIME',
                         'not_found':'TALK_FOUND',
                         'aborted':'ANNOUNCE_ANALYZE'}
        )
        
        #         smach.StateMachine.add('SETUP2', Setup(),
        #             transitions = {'succeeded':'HANDLE_NOT_FOUND_TIME'},
        #             remapping   = {'init_time':'init_time'}
        #         )
        
        smach.StateMachine.add('HANDLE_NOT_FOUND_TIME', HandleNotFoundTimed(),
            transitions={'continue-now':'TALK_FOUND',
                         'can-retry':'HANDLE_CROWDS'}
        )
        
        smach.StateMachine.add('EMOTION_RECOGNITION', EmotionRecognition.getInstance(),
            transitions = {'succeeded':'TALK_FOUND'},
            remapping = {'person_emotion':'operator_emotion'}
        )
        
        smach.StateMachine.add('TALK_FOUND',
            TalkState.getInstance('I found you operator', 2),
            transitions = {'succeeded':'TALK_INFO_OPERATOR'}
        )
        
        smach.StateMachine.add('TALK_INFO_OPERATOR', TalkInfoOperator(),
                transitions = {'succeeded':'TALK_INFO_CROWD'}
        )
         
        smach.StateMachine.add('TALK_INFO_CROWD', TalkInfoCrowd(),
                transitions = {'succeeded':'succeeded'},
                remapping   = {'n_men':'n_men',
                               'n_women':'n_women'}
        )
        
        # ud = smach.UserData()
        # sm.set_initial_state(['ANNOUNCE_SEEK'], ud)
        
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('person_recognition')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('person_recognition', sm, '/PERSON_RECOGNITION_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
