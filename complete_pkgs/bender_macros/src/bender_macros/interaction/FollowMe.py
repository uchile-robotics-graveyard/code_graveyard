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
from bender_arm_control.srv import HeadPosition, HeadPositionRequest, HeadPositionResponse
from bender_macros.speech import Talk
from bender_macros.speech import Recognize


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        Talk.getInstance('I will follow you',3)
        sp = "Please, stay in front of me and make small steps"
        Talk.getInstance(sp,len(sp)/8)

        return 'succeeded'

class TrainingPerson(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.training_person = rospy.ServiceProxy('/train_person', IsOn)
        self.follow_person = rospy.ServiceProxy('/Start_follow', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state TrainingPerson')

        try:
            self.training_person()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'


        sp = "Now, I will follow you... Please, say 'Bender stop follow me' to stop"
        Talk.getInstance(sp, len(sp)/8)

        try:
            self.follow_person()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        return 'succeeded'
        
class SearchInstructions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                 io_keys=['answer'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchInstructions')

        sentence = userdata.answer.split(" ")
        print sentence
        if len(sentence)<3:
            return 'aborted'

        c=0
        for sen in sentence:
            if "bender" in sen: c+=1
            if "stop" in sen: c+=1
            if "follow" in sen: c+=1
           
        if c > 2:
            return 'succeeded'
        return 'aborted'


class StopFollow(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.follow_person = rospy.ServiceProxy('/Stop_follow', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop Follow')

        try:
            self.follow_person()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        sp = "Now, I'll stop following you... Thanks for guiding me"
        Talk.getInstance(sp, len(sp)/8)

        return 'succeeded'


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.dicc = 'follow'  
    sm.userdata.answer = "" 


    with sm:

        smach.StateMachine.add('SETUP',Setup(),
            transitions={'succeeded':'TRAINING'}
        )

        smach.StateMachine.add('TRAINING', TrainingPerson(),
            transitions={'succeeded':'RECOGNIZE',
                         'aborted':'TRAINING'}
        )

        smach.StateMachine.add('RECOGNIZE',Recognize.getInstance(),
            transitions={'succeeded':'SEARCHINSTRUCTIONS'},
            remapping={'dictionary':'dicc',
                       'recognized_word':'answer'}
        )
        smach.StateMachine.add('SEARCHINSTRUCTIONS',SearchInstructions(),
            transitions={'succeeded':'STOP',
                         'preempted':'RECOGNIZE'}
        )
        smach.StateMachine.add('STOP',StopFollow(),
            transitions={'succeeded':'succeeded'}
        )
        
    return sm
 
 
# main
if __name__ == '__main__':

    rospy.init_node('FollowMe')

    sm = getInstance()
    ud = smach.UserData()
    ud.use_waist = False;
    # introspection server
    sis = smach_ros.IntrospectionServer('FollowMe', sm, '/FOLLOW_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
