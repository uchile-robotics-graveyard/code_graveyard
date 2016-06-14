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
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.vision import FindFace
from bender_macros.speech import Talk
from bender_macros.speech import AskForConfirmation
from bender_macros.nav import LookToPoseStamped
from bender_macros.head import MoveAsus


#from bender_macros.vision import 
from cv2 import imshow, waitKey
from bender_utils.ros import benpy

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'],
                                   input_keys = ['person_name'],
                                   output_keys = ['questiontext','questiontimeout']
)

        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):
        

        #Question parameters
        sp = 'are you ' + sm.userdata.person_name+' ?'
        sm.userdata.questiontext = sp
        sm.userdata.questiontimeout = len(sp)/8

        #self.talk_client.wait_for_service()

        # Put happy face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy1"
        self.face_pub.publish(emotion)#mover la cabeza a 0

        return 'succeeded'


class LookForCrowd(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['next','succeeded','preempted','aborted'],
                             output_keys=['pose_crowd','n_crowd'],
                             io_keys=['pose_persons_crowd','img_crowd','pose_persons_map'])
        
        self.person_det_enable_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)
        self.person_detect_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/get_detection', Recognition)
        self.c = 0
        self.max_eval = 1
        self.max_n = 0
        
    def execute(self, userdata):
                
        rospy.loginfo('Executing state LookForCrowd')
        Talk.getInstance("Starting Look For Crowd phase",3)
        while self.c < self.max_eval:
            
            # enable people detection
            self.person_det_enable_client(select=True)
            
            # wait for some recognitions
            rospy.sleep(3)
           # bridge = CvBridge()
            
            try:
                print "time :"+str(self.c)
                resp = self.person_detect_client()
                print "person detect :"+str(len(resp.pose))
                if len(resp.pose) > self.max_n:
                    # img = resp.imgs[0]
                    # cv_image = bridge.imgmsg_to_cv2(img, "32FC1")
                    # cv2.imshow("cv2",cv_image)

                    self.max_n = len(resp.pose)
                    userdata.pose_persons_crowd = resp.pose
                    userdata.img_crowd = resp.imgs

            except rospy.ServiceException, e:
                return 'preempted'

            self.c = self.c + 1

        # crowd size > 1
        if self.max_n >= 1:
            print "find pose"
            userdata.pose_persons_map = self.find_pose_map(userdata.pose_persons_crowd)
            #userdata.pose_crowd, userdata.pose_persons_crowd = self.find_pose(userdata.pose_persons_crowd)
            userdata.n_crowd = self.max_n
            self.person_det_enable_client(False)
            return 'succeeded'

        self.c = 0
        Talk.getInstance("Crowd not found",2)
        return 'next'

    # def find_pose(self, poses):
        
    #     transformer = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)

    #     x=-1.0
    #     ymin=-1.0
    #     ymax=-1.0
    #     z = 0


    #     poses_transformer = poses
    #     print "found " + str(len(poses))
    #     cont = 0
    #     for i in poses:
    #         try:
    #             req = TransformerRequest()

    #             req.pose_in = i
    #             req.frame_out = "bender/base_link"
    #             transf_out = transformer(req)
    #             ps = transf_out.pose_out
    #             poses_transformer[cont] = ps

    #             if x==-1 or ps.pose.position.x < x:
    #                 x=ps.pose.position.x 
    #             if ymin==-1 or ps.pose.position.y < ymin:
    #                 ymin=ps.pose.position.y
    #             if ymax==-1 or ps.pose.position.y > ymax:
    #                 ymax=ps.pose.position.y
    #             z = ps.pose.position.z
    #             cont += 0

    #         except rospy.ServiceException, e:
    #             print "Service call failed: %s"%e

    #     rep = geometry_msgs.msg.PoseStamped()

    #    # rep = poses_transformer[0]
        
    #     rep.pose.position.x = x - (ymax -ymin)/(2*math.tan(58/2))-0.01    
    #     rep.pose.position.y = ymin + (ymax -ymin)/2
    #     rep.pose.position.z = z
    #     rep.header.stamp = rospy.Time.now()
    #     rep.pose.orientation.w = 1
    #     rep.header.frame_id = poses_transformer[0].header.frame_id
        
    #     #TODO definir orientacion

        
    #     print rep

    #     return rep, poses_transformer

    def find_pose_map(self,poses):
    
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

class SEARCH_PERSON(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                             input_keys = ['person_name','pose_persons_crowd','n_crowd','pose_persons_map'], 
                             io_keys = ['person_pose','person_mapose','asked_persons'])
       
    def execute(self, userdata):
        # if number of asked persons < total number of persons in the crowd
        # then go to the next person in the array and ask
        if userdata.asked_persons < userdata.n_crowd:
            # userdata.person_pose = userdata.pose_persons_crowd[userdata.asked_persons]
            userdata.person_mapose = userdata.pose_persons_map[userdata.asked_persons]
            if userdata.pose_persons_crowd[userdata.asked_persons] is None or userdata.pose_persons_map[userdata.asked_persons] is None:
                return 'preempted'
            userdata.asked_persons+= 1
            return 'succeeded'

        # Person not in the crowd text
        thugtext = 'no person named ' + userdata.person_name + ' is in the crowd'
        Talk.getInstance(thugtext,(len(thugtext)/8))

        return 'aborted'

class PERSON_FOUND(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys = ['person_name']
    )
       
    def execute(self, userdata):

        rospy.loginfo('I FOUND THE RIGHT PERSON')        
        foundtext = 'Hi ' + userdata.person_name + '!' 
        Talk.getInstance(foundtext, ( len(foundtext)/8 ) )
        # Put happy face
        #emotion = Emotion()
        #emotion.Order = "changeFace"
        #emotion.Action = "happy1"
        return 'succeeded'

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
        input_keys = ['person_name'])
    
    
    # - - -  parameters - - -
    # pre-enroll speech
    sm.userdata.wait_text = [
         "i dont't see anyone, can you step in front of me?",
         "can you step closer, please?"]
    sm.userdata.wait_timeout = -1
    
    # enroll images to save
    sm.userdata.n_enroll_images = 10
    
    # angle to turn after memorizing person 
    sm.userdata.turn_around_angle = 180

    # crowd poses
    sm.userdata.pose_persons_crowd = []
    
    
    sm.userdata.deg_angle = 20
    
    sm.userdata.n_crowd = 0
    sm.userdata.n_crowdW = 0
    sm.userdata.n_crowdM = 0

    sm.userdata.img_crowd = []
    sm.userdata.state_crowd = []
    sm.userdata.person_pose = PoseStamped()
    sm.userdata.person_mapose = PoseStamped()

    #number of persons asked
    sm.userdata.asked_persons = 0


    # Fill Machine
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                transitions = {'succeeded':'MOVE_ASUS'},
                remapping = {'questiontext':'questiontext',
                             'questiontimeout':'questiontimeout'}
        )

        #Estado para preguntar nombre de persona a buscar
        #smach.StateMachine.add('ASK_OPERATOR_FOR_NAME',ASK_OPERATOR_FOR_NAME(),
        #   transitions = {'succeded':'LOOK_FOR_CROWD','preempted':'ASK_OPERATOR_FOR_NAME'},
        #   remapping = {'person_name':'person_name'}
        #)
        smach.StateMachine.add('MOVE_ASUS',MoveAsus.getInstance(),
                transitions={'succeeded':'LOOK_FOR_CROWD'},
                remapping={'deg_angle':'deg_angle'})

        smach.StateMachine.add('LOOK_FOR_CROWD', LookForCrowd(),
                transitions = {'succeeded':'SEARCH_PERSON',
                                'preempted':'LOOK_FOR_CROWD',
                                'next':'LOOK_FOR_CROWD'},
                remapping = {'pose_crowd':'pose_crowd',
                             'n_crowd':'n_crowd',
                             'pose_persons_crowd':'pose_persons_crowd'}
        )
    
        smach.StateMachine.add('SEARCH_PERSON',SEARCH_PERSON(),
                transitions = {'succeeded':'APPROACH_TO_PERSON',
                                'aborted':'aborted',
                                'preempted':'APPROACH_TO_PERSON'},
                remapping = {'person_pose':'person_pose',
                            'n_crowd':'n_crowd',
                             'pose_persons_crowd':'pose_persons_crowd',
                             'asked_persons':'asked_persons',
                             'person_mapose':'person_mapose',
                             'pose_persons_map':'pose_persons_map'}
        )

        smach.StateMachine.add('APPROACH_TO_PERSON', ApproachToPoseStamped.getInstance(),
                transitions = {'succeeded':'LOOK_TO_PERSON',
                            'aborted':'APPROACH_TO_PERSON'},
                remapping = {'goal_pose':'person_mapose'}
        ) 

        smach.StateMachine.add('LOOK_TO_PERSON', LookToPoseStamped.getInstance(),
                transitions = {'succeeded':'CONFIRM_PERSON','aborted':'LOOK_TO_PERSON',
                                'preempted':'APPROACH_TO_PERSON'},
                remapping = {'goal_pose':'person_mapose'}
        )

        #Si hay dos Susan seria muy troll, no se toma ese caso aqui
        smach.StateMachine.add('CONFIRM_PERSON',AskForConfirmation.getInstance(),
                transitions={'yes':'PERSON_FOUND',
                             'no':'SEARCH_PERSON'},
                remapping = {'text':'questiontext',
                            'timeout':'questiontimeout'}
        )

        smach.StateMachine.add('PERSON_FOUND',PERSON_FOUND(),
                transitions = {'succeeded':'succeeded'})
      
        

        ud = smach.UserData()

        #ud.person_pose = person_pose
        
       # sm.set_initial_state(['SETUP'], ud)

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('person_search')

    sm = getInstance()
    ud = smach.UserData()
    ud.person_name = "andres"
    
    # introspection server
    sis = smach_ros.IntrospectionServer('person_search', sm, '/PERSON_SEARCH_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
