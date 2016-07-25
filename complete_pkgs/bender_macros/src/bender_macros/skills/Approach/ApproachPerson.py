#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros


from bender_srvs.srv import *
from bender_msgs.msg import *
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

from bender_macros.speech import Talk
from bender_macros.speech import AskForConfirmation
from bender_macros import LookForSomeFace
from bender_macros.nav import ApproachToPoseStamped,RotateRobot
from bender_macros.nav import ApproachToPlace
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import GoToPlace
from bender_macros.head import MoveAsus
from bender_utils.ros import benpy



class Start(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.detector_enable_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)

    def execute(self, userdata):
        try:
            self.detector_enable_client(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'



class LookForPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','cancel','notfound','rotate','aborted','preempted'],
                             output_keys=['sm_person_pose','report'])
    
        self.detector_enable_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/enable', Onoff)
        self.detector_get_client = benpy.ServiceProxy('/bender/pcl/hog_person_detector/get_detection', Recognition)
        self.room_detection_client = benpy.ServiceProxy('/bender/nav/map_analyzer/check_point_inside_map', ValidPoint)

        self.count = 0 
        self.max_count = 3
        self.max_count_finish = 12

    def execute(self, userdata):
        rospy.loginfo('Executing state LookForPerson')
        
        self.count +=1
        if self.count > self.max_count_finish : 
            userdata.report = "I dont found any person"
            return 'cancel'
        if self.count % self.max_count == 0 : 
            return 'rotate'

        try:
            xyzcamera = self.detector_get_client()
            if len(xyzcamera.pose)==0:
                return 'notfound'
            for i in xyzcamera.pose:
                if self.CheckPose(i):
                    userdata.sm_person_pose = i
            self.detector_enable_client(False)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'
         
        Talk.getInstance('I saw a person. I am going', 3)

        return 'succeeded'

    def CheckPose(self, person_pose):

        req = ValidPointRequest()
        req.point.x = person_pose.pose.position.x
        req.point.y = person_pose.pose.position.y
        req.point.z = person_pose.pose.position.z
        req.frame_id = person_pose.header.frame_id
        print req
        resp = self.room_detection_client (req)
        print resp
        return resp.is_valid


class TransformPose(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             io_keys=['sm_person_pose'])
        self.transform_client = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)

    def execute(self, userdata):
        rospy.loginfo('Executing state TransformPose')

        print userdata.sm_person_pose
        tf_req = TransformerRequest()
        tf_req.pose_in = userdata.sm_person_pose
        tf_req.frame_out = "map"
        transf_out = self.transform_client(tf_req)
        userdata.sm_person_pose = transf_out.pose_out

        print userdata.sm_person_pose
        return 'succeeded'

class AskName(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','confirm','aborted','preempted'],
                input_keys = ['person_name','text']
        )
        
        
    def execute(self, userdata):
        
        name = "person"
        if not userdata.person_name == "person":
            name = userdata.person_name
            userdata.text = 'Hi, are you '+ name +"?"
            return 'confirm'

        Talk.getInstance('Hi, person ', 2)
                
        return 'succeeded'

                    
class AskMove(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.loginfo('Recalculating Goal')
        
        sp = "Sorry, I'm looking for someone else"
        Talk.getInstance(sp, len(sp)/9)
        
        sp = "can you move? please"
        Talk.getInstance(sp, len(sp)/9)

        return 'succeeded'     



# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  M a c h i n e    C r e a t i o n                     #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #
#TODO Implementar que este dentro del mapa
def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','notfound','aborted','preempted'],
        input_keys=['person_name'],
        output_keys=['report']
        )

    sm_listen = smach.StateMachine(outcomes=['succeeded'])

    sm.userdata.sm_person_pose = PoseStamped()
    sm.userdata.found_talk = 'I have found you'
    sm.userdata.timeout_talk = 4

    sm.userdata.report = "I found a person"

    sm.userdata.deg_angle =20

    sm.userdata.rotate_angle = 50
    sm.userdata.rotate_angle_move = 180

    with sm:


        smach.StateMachine.add('START_STATE',Start(),
                       transitions={'succeeded':'MOVEASUS',
                                    'aborted':'START_STATE'})
        
        smach.StateMachine.add('MOVEASUS',MoveAsus.getInstance(),
            transitions={'succeeded':'LOOK_FOR_PERSON'},
            remapping={'deg_angle':'deg_angle'}
            )

        smach.StateMachine.add('LOOK_FOR_PERSON', LookForPerson(),
            transitions = {
                'succeeded':'TRANSFORM',
                'notfound' :'LOOK_FOR_PERSON',
                'cancel' :'notfound',
                'rotate' : 'TURN_AROUND',
                'aborted':'LOOK_FOR_PERSON'
            }
        )
        smach.StateMachine.add('TRANSFORM',TransformPose(),
                transitions={'succeeded':'APPROACH_TO_PERSON'}
        )
        smach.StateMachine.add('TURN_AROUND',RotateRobot.getInstance(),
            transitions = {'succeeded':'MOVEASUS',
                           'aborted':'MOVEASUS'},
            remapping   = {'angle':'rotate_angle'}
        )
        smach.StateMachine.add('APPROACH_TO_PERSON', ApproachToPoseStamped.getInstance(),
           transitions = {'succeeded':'LOOK_PERSON','aborted':'LOOK_FOR_PERSON'},
           remapping = {'goal_pose':'sm_person_pose'}
        )
        smach.StateMachine.add('LOOK_PERSON', LookToPoseStamped.getInstance(),
           transitions = {'succeeded':'succeeded','aborted':'LOOK_FOR_PERSON'},
           remapping = {'goal_pose':'sm_person_pose'}
        )
        smach.StateMachine.add('AskName', AskName(),
            transitions = {'confirm':'CONFIRM',
                            'succeeded':'succeeded'},
            remapping = {'text':'found_talk','timeout':'timeout_talk'}
        )
        smach.StateMachine.add('CONFIRM',AskForConfirmation.getInstance(),
                           transitions={'yes':'succeeded',
                                        'no':'AskMove'},
                           remapping={'text':'answer',
                                    'timeout':'timeout'}
                           )
        smach.StateMachine.add('AskMove', AskMove(),
            transitions = {'succeeded':'TURN_AROUND_MOVE'}
        )
        smach.StateMachine.add('TURN_AROUND_MOVE',RotateRobot.getInstance(),
            transitions = {'succeeded':'LOOK_FOR_PERSON',
                           'aborted':'LOOK_FOR_PERSON'},
            remapping   = {'angle':'rotate_angle_move'}
        )
        
        
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('look_person')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('look_person', sm, '/LOOK_PERSON_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
