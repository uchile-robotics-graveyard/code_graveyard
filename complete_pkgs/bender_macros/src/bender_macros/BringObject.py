#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf

from bender_msgs.msg import Emotion

# macros
from bender_macros import GoForObject 
from bender_macros.nav import GoToPoseStamped

# services
import bender_srvs.srv

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class foo(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'])
        
    def execute(self, userdata):

        rospy.loginfo('Executing state: foo')
        rospy.sleep(1)
        
        return 'succeeded'

class AnalizeResult(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','not_found','not_grabbed','aborted','preempted'],
            input_keys=['result']
        )
        
    def execute(self, userdata):
        return userdata.result

class ActSucceeded(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['result']
        )

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',bender_srvs.srv.synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
        
    def execute(self, userdata):

        self.talk_client.wait_for_service()

        # Put happy face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy3"
        self.face_pub.publish(emotion)

        # anounce ready
        text = 'i do it'
        self.talk_client(text)
        rospy.sleep(2)

        # Put happy face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy1"
        self.face_pub.publish(emotion)

        userdata.result = 'succeeded'
        return 'succeeded'

class ActNotFound(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['result']
        )

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',bender_srvs.srv.synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):

        self.talk_client.wait_for_service()

        # Put sad face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "sad1"
        self.face_pub.publish(emotion)

        # anounce ready
        text = 'i cant see the object'
        self.talk_client(text)
        rospy.sleep(2)
        
        userdata.result = 'not_found'
        return 'succeeded'

class ACT_NOT_GRABBED(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['result']
        )

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',bender_srvs.srv.synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):

        self.talk_client.wait_for_service()

        # Put ashamed face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "ashamed"
        self.face_pub.publish(emotion)

        # anounce ready
        text = 'i cant grasp the object'
        self.talk_client(text)
        rospy.sleep(2)

        # Put ashamed face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "sad2"
        self.face_pub.publish(emotion)

        userdata.result = 'not_grabbed'
        return 'succeeded'

class GetCurrentPosition(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['current_pose']            
        )
        
        self.get_current_pose_client = rospy.ServiceProxy(
            '/bender/nav/goal_server/get_current_pose',bender_srvs.srv.PoseStamped
        )
        
        self.transform_client = rospy.ServiceProxy(
            '/bender/tf/simple_pose_transformer/transform',bender_srvs.srv.Transformer
        )
        
    def execute(self, userdata):

        rospy.loginfo('Executing state: GET_CURRENT_POSITION')

        self.get_current_pose_client.wait_for_service()
        self.transform_client.wait_for_service()
        
        try:
            curr_pose_res = self.get_current_pose_client()
            
            transform_res = self.transform_client(
                pose_in=curr_pose_res.pose_out,
                frame_out='/map'
            )
            
            userdata.current_pose = transform_res.pose_out
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'
        
        return 'succeeded'
    
# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    
    sm = smach.StateMachine(
        outcomes=['succeeded','not_found','not_grabbed','aborted','preempted'],
        input_keys=['object_name'],
        output_keys=['grasp_arm','result']
    )
    
    #sm.userdata.initial_pose
    sm.userdata.grasp_arm = ''
    sm.userdata.result = ''

    with sm:

        smach.StateMachine.add('GET_CURRENT_POSITION', GetCurrentPosition(),
            transitions = {'succeeded':'GO_FOR_OBJECT'},
            remapping = {'current_pose':'initial_pose'}
        )

        smach.StateMachine.add('GO_FOR_OBJECT',GoForObject.getInstance(),
            transitions = {'succeeded':'ACT_SUCCEEDED',
                           'not_grabbed':'ACT_NOT_GRABBED',
                           'not_found':'ACT_NOT_FOUND'},
            remapping = {'object_name':'object_name',
                         'grasp_arm':'grasp_arm'}
        )

        smach.StateMachine.add('ACT_SUCCEEDED',ActSucceeded(),
            transitions = {'succeeded':'RETURN'},
            remapping = {'result':'result'}
        )
        
        smach.StateMachine.add('ACT_NOT_FOUND',ActNotFound(),
            transitions = {'succeeded':'RETURN'},
            remapping = {'result':'result'}
        )
 
        smach.StateMachine.add('ACT_NOT_GRABBED',ACT_NOT_GRABBED(),
            transitions = {'succeeded':'RETURN'},
            remapping = {'result':'result'}
        )
        
        smach.StateMachine.add('RETURN',GoToPoseStamped.getInstance(),
            transitions = {'succeeded':'ANALIZE_RESULT'},
            remapping = {'goal_pose':'initial_pose'}
        )

        smach.StateMachine.add('ANALIZE_RESULT',AnalizeResult(),
                transitions = {'succeeded':'succeeded',
                               'not_found':'not_found',
                               'not_grabbed':'not_grabbed'},
                remapping = {'result':'result'}
        )
        
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('bring_object')

    sm = getInstance()

    ud = smach.UserData()
    ud.object_name = 'cola'

    # introspection server
    sis = smach_ros.IntrospectionServer('bring_object', sm, '/BRING_OBJECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
