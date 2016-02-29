#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from bender_macros import LookForSomeFace
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.nav import ApproachToPlace
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import GoToPlace

from geometry_msgs.msg import PoseStamped
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import SearchPerson
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_srvs.srv import stringReturn
from bender_srvs.srv import synthesize
from std_srvs.srv import Empty

class talk(smach.State):

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
    
class foo(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.sleep(0.5)
        return 'succeeded'
    
class recalculateGoal(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
    def execute(self, userdata):
        rospy.loginfo('Recalculating Goal')
        try:
            re = rospy.ServiceProxy('bender/nav/recalculateGoal', Empty)
            output = re()
            return 'succeeded'
        except rospy.ServiceException, e:
            rospy.WARN('Service call failed: %s'%e)            

class interact(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['understand'])
        
    def execute(self, userdata):
        rospy.loginfo('Listening the questions')
        try:
            wdys = rospy.ServiceProxy('bender/speech/interaction/answer_request', stringReturn)
            output = wdys()
            return 'understand'
        except rospy.ServiceException, e:
            return 'understand'

class askagain(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','continue'])
        self.counter = 1
        
    def execute(self, userdata):
        while self.counter < 3:
            self.counter += 1
            rospy.loginfo('%s%d' % ('Question number: ',self.counter))
            return 'continue'
        
        return 'succeeded'
                
                    

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  M a c h i n e    C r e a t i o n                     #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'])

    sm_listen = smach.StateMachine(outcomes=['succeeded'])

    sm.userdata.sm_person_pose = PoseStamped()
    sm.userdata.found_talk = 'I have found you'
    sm.userdata.saw_talk = 'I saw a person. I am going'
    sm.userdata.timeout_talk = 4

    with sm:

        def detect_face_response_cb(userdata, response):
            rospy.loginfo('number of detected faces: ' + str(response.n_faces))
            if response.n_faces > 0:
                return 'succeeded'
            else:
                return 'aborted'

        smach.StateMachine.add('START_STATE',foo(),
                       transitions={'succeeded':'LOOK_FOR_SOME_FACE'})

        smach.StateMachine.add('LOOK_FOR_SOME_FACE', LookForSomeFace.getInstance(),
            transitions = {
                'succeeded':'TALK1',
                'aborted':'LOOK_FOR_SOME_FACE'
            },
            remapping = {
                'sm_face_pose':'sm_person_pose'
            }
        )
        smach.StateMachine.add('TALK1', talk(),
            transitions = {
                  'succeeded':'APPROACH_TO_PERSON',
                  'aborted':'TALK1'
                  },
            remapping = {'text':'saw_talk',
                     'timeout':'timeout_talk'}
        )

        smach.StateMachine.add('APPROACH_TO_PERSON', ApproachToPoseStamped.getInstance(),
           transitions = {'succeeded':'RECALCULATE_2','aborted':'LOOK_FOR_SOME_FACE'},
           remapping = {'goal_pose':'sm_person_pose'}
        )
        smach.StateMachine.add('RECALCULATE_2', smach_ros.ServiceState('bender/nav/recalculateGoal',Empty),
            transitions={'succeeded':'LOOK_PERSON'})
        
        smach.StateMachine.add('LOOK_PERSON', LookToPoseStamped.getInstance(),
           transitions = {'succeeded':'DETECT_FACE','aborted':'LOOK_FOR_SOME_FACE'},
           remapping = {'goal_pose':'sm_person_pose'}
        )
        
        smach.StateMachine.add('TALK2', talk(),
            transitions = {'succeeded':'LISTEN','aborted':'TALK2'},
            remapping = {'text':'found_talk','timeout':'timeout_talk'}
        )
        
        smach.StateMachine.add('DETECT_FACE',
                smach_ros.ServiceState('/bender/vision/face_detector/detect_face',
                FaceInfo,
                response_cb = detect_face_response_cb
            ),
            transitions = {'succeeded':'TALK2','aborted':'LOOK_FOR_SOME_FACE'},
            remapping = {'person_pose':'sm_person_pose'}
        )        
        
        with sm_listen:
            
            smach.StateMachine.add('INTERACTION',interact(),
                transitions = {'understand':'ASKAGAIN'}
            )
            
            smach.StateMachine.add('ASKAGAIN',askagain(),
                transitions = {
                    'succeeded':'succeeded',
                    'continue':'INTERACTION'
                }
            )
            
        smach.StateMachine.add('LISTEN',sm_listen,
           transitions={
                'succeeded':'succeeded'
            }
        )
        
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('what_did_you_say')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('what_did_you_say', sm, '/WHAT_DID_YOU_SAY_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
