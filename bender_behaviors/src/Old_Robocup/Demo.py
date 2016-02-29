#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from bender_msgs.msg import Emotion
from bender_srvs.srv import stringReturn
from bender_srvs.srv import synthesize
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import SearchPerson
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_srvs.srv import stringReturn
from bender_srvs.srv import Onoff

#  - - - - macros - - - -
from bender_macros.nav import GoToPlace
from bender_macros.nav import ApproachToPoseStamped


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n 
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):
        
        # Put happy face
        emotion = Emotion()
        emotion.Order = "changeFace"
        emotion.Action = "happy1"
        self.face_pub.publish(emotion)

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

class WaitForPerson(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted']
        )
    
        self.blob_subs = rospy.Subscriber("/blob/filter", Bool, self.detect_cb)
        self.cnt = 0

    def execute(self, userdata):

        while (self.cnt < 10):
            rospy.sleep(0.1)

        return 'succeeded'

    def detect_cb(msg):

        if msg.data:
            self.cnt += 1

class ShowFace(smach.State):

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted']
        )

    def execute(self, userdata):

        client = rospy.ServiceProxy('/showface',Onoff)
        rospy.sleep(2)
        print "waiting for service /showface"
        client.wait_for_service()
        print "wait done"

        client(select=True)

        return 'succeeded'

    def detect_cb(msg):

        if msg.data:
            self.cnt += 1

def getMachine():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    sm.userdata.entrance_place = 'door_left_post_in'
    sm.userdata.master_room = 'demo_master_room'
    
    sm.userdata.map_name = 'map.sem_map'
    
    sm.userdata.tts_0 = 'master . . . . . .  there is someone waiting at the door'
    sm.userdata.tts_0_timeout = 10
    sm.userdata.tts_qx_1 = 'Should i let him get in the house . . . . master?'
    sm.userdata.tts_qx_1_timeout = 11
    sm.userdata.tts_rx_1 = 'Sorry . . . . . . . my master told me you cant come In. . . . . . . . . . . So  you shall not pass'
    sm.userdata.tts_rx_1_timeout = 20
    sm.userdata.tts_qx_2 = 'he has not leave yet . . master . . . . . . . what should i do?'
    sm.userdata.tts_qx_2_timeout = 11
    sm.userdata.tts_rx_2 = 'its OK . . . .  please follow me'
    sm.userdata.tts_rx_2_timeout = 5
   
    # Sub Machines - States
    go_to_place_sm = GoToPlace.getInstance()
    approach_to_pose_sm =  ApproachToPoseStamped.getInstance()
        
    # Fill Machine
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                transitions = {'succeeded':'WAIT_FOR_PERSON'}
        )
        smach.StateMachine.add('WAIT_FOR_PERSON',WaitForPerson(),
                transitions = {'succeeded':'TALK_0'}
        )
        smach.StateMachine.add('TALK_0',Talk(),
                transitions = {'succeeded':'GO_TO_ENTRANCE'},
                remapping = {'text':'tts_0',
                             'timeout':'tts_0_timeout'}
        )
        smach.StateMachine.add('GO_TO_ENTRANCE', go_to_place_sm,
                transitions = {'succeeded':'SHOW_FACE'},
                remapping = {'place_name':'entrance_place',
                             'map_name':'map_name'}
        )
        smach.StateMachine.add('SHOW_FACE',ShowFace(),
                transitions = {'succeeded':'TALK_QX_1'}
        )
        smach.StateMachine.add('TALK_QX_1',Talk(),
                transitions = {'succeeded':'TALK_RX_1'},
                remapping = {'text':'tts_qx_1',
                             'timeout':'tts_qx_1_timeout'}
        )
        smach.StateMachine.add('TALK_RX_1',Talk(),
                transitions = {'succeeded':'TALK_QX_2'},
                remapping = {'text':'tts_rx_1',
                             'timeout':'tts_rx_1_timeout'}
        )
        smach.StateMachine.add('TALK_QX_2',Talk(),
                transitions = {'succeeded':'TALK_RX_2'},
                remapping = {'text':'tts_qx_2',
                             'timeout':'tts_qx_2_timeout'}
        )
        smach.StateMachine.add('TALK_RX_2',Talk(),
                transitions = {'succeeded':'RETURN_TO_MASTER_ROOM'},
                remapping = {'text':'tts_rx_2',
                             'timeout':'tts_rx_2_timeout'}
        )
        smach.StateMachine.add('RETURN_TO_MASTER_ROOM', go_to_place_sm,
                transitions = {'succeeded':'succeeded'},
                remapping = {'place_name':'demo_master_room',
                             'map_name':'map_name'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('cocktail_party')

    sm = getMachine()

    # introspection server
    sis = smach_ros.IntrospectionServer('demo_challenge', sm, '/DEMO_CHALLENGE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
