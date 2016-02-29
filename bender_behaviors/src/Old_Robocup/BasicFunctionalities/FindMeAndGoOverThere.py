#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped

from bender_macros.vision import EnrollFace


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


class WaitInitSignal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: WAIT_INIT_SIGNAL')
        rospy.sleep(0.5)
        return 'succeeded'


class SearchPerson(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'],
                             input_keys=['face_id'],
                             output_keys=['person_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: SEARCH_PERSON')
        rospy.sleep(0.5)

        rospy.loginfo('searching person ' + str(userdata.face_id) ) 
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = '/hello_world_frame'
        userdata.person_pose = pose_stamped

        return 'succeeded'


class ApproachTo(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'],
                             input_keys=['person_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: APPROACH_TO_PERSON')
        rospy.sleep(0.5)

        rospy.loginfo('approaching to person pose on frame: '
         + userdata.person_pose.header.frame_id ) 

        return 'succeeded'


class PointingGesture(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: POINTING_GESTURE')
        rospy.sleep(0.5)
        return 'succeeded'



# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  M a c h i n e    C r e a t i o n                     #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # sm variables
    sm.userdata.sm_current_face_id = -1
    sm.userdata.sm_current_person_pose = PoseStamped()

    with sm:

        # Falta el inicio:
        #     - buscar alguna cara, acercarse y mirar
        # or  - pedir acercamiento


        sm_enroll = EnrollFace.getInstance()

        smach.StateMachine.add('ENROLL_FACE',sm_enroll,
                           transitions={'succeeded':'WAIT_INIT_SIGNAL',
                                        'aborted':'aborted',
                                        'preempted':'preempted',
                                        'srv_except':'aborted'},
                           remapping={'enroll_face_id':'sm_current_face_id'})

        smach.StateMachine.add('WAIT_INIT_SIGNAL',WaitInitSignal(),
                           transitions={'succeeded':'SEARCH_PERSON',
                                        'aborted':'aborted'})

        smach.StateMachine.add('SEARCH_PERSON',SearchPerson(),
                           transitions={'succeeded':'APPROACH_TO_PERSON',
                                        'aborted':'aborted'},
                           remapping={'face_id':'sm_current_face_id',
                                      'person_pose':'sm_current_person_pose'})

        smach.StateMachine.add('APPROACH_TO_PERSON',ApproachTo(),
                           transitions={'succeeded':'POINTING_GESTURE',
                                        'aborted':'aborted'},
                           remapping={'person_pose':'sm_current_person_pose'})

        smach.StateMachine.add('POINTING_GESTURE',PointingGesture(),
                           transitions={'succeeded':'succeeded',
                                        'aborted':'aborted'})

    return sm



# main
if __name__ == '__main__':

    rospy.init_node('find_me_and_go_over_there')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('find_me_and_go_over_there', sm, '/FIND_ME_AND_GO_OVER_THERE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
