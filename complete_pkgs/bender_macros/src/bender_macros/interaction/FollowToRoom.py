#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from math import pi
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *
from geometry_msgs.msg import *
from bender_arm_control.srv import HeadPosition, HeadPositionRequest, HeadPositionResponse
from bender_macros.speech import Talk
from bender_macros.speech import TalkState
from bender_utils.ros import benpy
# from bender_macros.speech import Recognize


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class TrainingPerson(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.training_person = rospy.ServiceProxy('/train_person', IsOn)

    def execute(self, userdata):
        rospy.loginfo('Executing state TrainingPerson')

        try:
            self.training_person()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'

class FollowInit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                   input_keys=['room'])
        self.follow_person = rospy.ServiceProxy('/Start_follow', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state INIT_FOLLOW')

        Talk.getInstance("Now, I follow you to the " + userdata.room, 4)

        try:
            self.follow_person()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'

class FollowStop(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.Stop_follow_client = rospy.ServiceProxy('/Stop_follow', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop Follow')

        try:
            self.Stop_follow_client()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'

        
class CompareRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['same','other'],
                                   input_keys=['room'])

        self.bender_pose = rospy.Subscriber('/bender/nav/robot_pose_publisher/pose', geometry_msgs.msg.PoseStamped, self.bender_pose_map)
        self.room_detection_client = benpy.ServiceProxy('/bender/nav/map_analyzer/check_point_inside_map', ValidPoint)
        self.pose = geometry_msgs.msg.PoseStamped()

    def bender_pose_map(self, msg):
        self.pose = msg

    def find_room(self,msg):
        req = ValidPointRequest()
        req.point.x = msg.pose.position.x
        req.point.y = msg.pose.position.y
        req.point.z = msg.pose.position.z
        req.frame_id = msg.header.frame_id

        resp = self.room_detection_client (req)
        return resp.is_valid, resp.data

    def execute(self, userdata):
        rospy.loginfo('Executing state CompareRoom')

        while (self.pose.pose.position.x == 0 and self.pose.pose.position.y == 0 and self.pose.pose.position.z == 0):
            rospy.sleep(1)

        valid,rooms = self.find_room(self.pose)
        if userdata.room in rooms and valid:
            sp = "We  "+userdata.room
            Talk.getInstance(sp, len(sp)/8)
            return 'same'

        rospy.sleep(1)       
        return 'other'


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['room'])

    with sm:

        smach.StateMachine.add('ANNOUNCE_FOLLOW',
            TalkState.getInstance('I will follow you... please stay in front of me and make small steps', 6),
            transitions = {'succeeded':'TRAIN_OPERATOR'}
        )

        smach.StateMachine.add('TRAIN_OPERATOR', TrainingPerson(),
                transitions={'succeeded':'INIT_FOLLOW',
                             'aborted':'TRAIN_OPERATOR'}
        )

        smach.StateMachine.add('INIT_FOLLOW', FollowInit(),
                transitions={'succeeded':'COMPAREROOM',
                             'aborted':'TRAIN_OPERATOR'}
        )

        smach.StateMachine.add('COMPAREROOM', CompareRoom(),
                transitions = {'same':'STOP_FOLLOW',
                               'other':'COMPAREROOM'},
                remapping   = {'room':'room'})

        smach.StateMachine.add('STOP_FOLLOW', FollowStop(),
               transitions={'succeeded':'ANNOUNCE_STOP'})

        smach.StateMachine.add('ANNOUNCE_STOP',
            TalkState.getInstance("i'll stop following you... Thanks for guiding me", 4),
            transitions = {'succeeded':'succeeded'}
        )

        return sm
 
 
# main
if __name__ == '__main__':

    rospy.init_node('FollowToRoom')

    sm = getInstance()
    ud = smach.UserData()
    ud.room = "livingroom";

    # introspection server
    sis = smach_ros.IntrospectionServer('FollowToRoom', sm, '/FOLLOWTOROOM_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
