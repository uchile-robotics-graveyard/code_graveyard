#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import datetime
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_behaviors.Stage1.GPSR import ParseOrder
from bender_behaviors.Stage1.GPSR import Actions
from bender_behaviors.Stage1.GPSR import GPSR_keyboard
from bender_macros.speech import AskGPSR    
from bender_macros.speech import Talk
from bender_macros.speech import TalkState
from bender_macros.nav import GoToPlace
from bender_macros.vision import FindFace
from bender_macros.speech import AskForConfirmation


class CoutingTimes(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['normal','team','keyboard','succeeded','aborted','preempted'])
        self.n = 0
    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        self.n +=1

        if self.n <=3:
            return 'normal'
        if self.n ==4:
            return 'team'

        return 'keyboard'


class ResetInf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
        output_keys=['question','request_person','request_place','request_object','request_action','request_nactions'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ResetInf')
        userdata.request_nactions = 0
        userdata.request_person = []
        userdata.request_place = []
        userdata.request_object = []
        userdata.request_action = []
        userdata.question = ""
        


        return 'succeeded'

class EvaluateGPSR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','next'],
        input_keys=['request_person','request_place','request_object','request_action','request_nactions'],
                             io_keys=['question'])
        self.count=0

    def execute(self, userdata):
        rospy.loginfo('Executing state EvaluateGPSR')
        if self.count == 0 : userdata.question = "do you want me to "
        else: 
            if self.count == userdata.request_nactions-1:
                userdata.question +=" and "
            else:   userdata.question +=", "

        i = self.count
        self.count+=1
        if self.count == (userdata.request_nactions+1):
            self.count = 0
            return 'succeeded'

        if userdata.request_action[i] == "go" or userdata.request_action[i] == "aproach":
            if not userdata.request_place[i] == "me":
                userdata.question+="go to the "+userdata.request_place[i]
            else:
                userdata.question+="come to you"
        if userdata.request_action[i] == "find person":  userdata.question+="find a person"
        if userdata.request_action[i] == "waving":  userdata.question+="find the waving person"
        if userdata.request_action[i] == "find object":  
            userdata.question+="find the "+userdata.request_object[i]+" in the "+userdata.request_place[i]
        if userdata.request_action[i] == "follow":      userdata.question+="follow a person"
        if userdata.request_action[i] == "count object":   userdata.question+="count objects"
        if userdata.request_action[i] == "report":  userdata.question+="report to you"
        if userdata.request_action[i] == "grasp": userdata.question+="grasp the "+userdata.request_object[i]
        if userdata.request_action[i] == "bring": userdata.question+="find the "+userdata.request_object[i] 
        if userdata.request_action[i] == "offer":  userdata.question+="offer a "+userdata.request_object[i]
        if userdata.request_action[i] == "take this": userdata.question+="receive the object"
        if userdata.request_action[i] == "answer":  userdata.question+="answer him a question"
        if userdata.request_action[i] == "give": userdata.question+="give the object"
        if userdata.request_action[i] == "place": userdata.question+="place the object"
        if userdata.request_action[i] == "tell name team":  userdata.question+="tell the team name"
        if userdata.request_action[i] == "tell name":   userdata.question+="tell him my name"
        if userdata.request_action[i] == "tell time":   userdata.question+="tell him the time"
        if userdata.request_action[i] == "tell date":   userdata.question+="tell him the date"
        if userdata.request_action[i] == "tell day today":  userdata.question+="tell him the day"
        if userdata.request_action[i] == "tell day tomorrow":   userdata.question+="tell him the day of tomorrow"
        if userdata.request_action[i] == "tell day month":  userdata.question+="tell him the day of the month"
        if userdata.request_action[i] == "tell day week":   userdata.question+="tell him the day of the week"
        if userdata.request_action[i] == "introduce":   userdata.question+="introduce my self"
        if userdata.request_action[i] == "ask name":   userdata.question+="ask her name"
        if userdata.request_action[i] == "ask lastname":   userdata.question+="ask her lastname"
        if userdata.request_action[i] == "ask nickname":   userdata.question+="ask her nickname"

        return 'next'
# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    sm.userdata.map_name = 'map.sem_map'

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sm.userdata.avoid_that_place_name = 'wdys'
    sm.userdata.pre_avoid_that_place_name = 'avoid_that'
    sm.userdata.wdys_place = 'kitchen'
    sm.userdata.init_place = 'livingroom'
    sm.userdata.exit_pre  = 'exit_pre'
    sm.userdata.exit_post = 'exit_post'

    sm.userdata.request_instructions = ""
    sm.userdata.request_nactions = 0
    sm.userdata.request_person = []
    sm.userdata.request_place = []
    sm.userdata.request_object = []
    sm.userdata.request_action = []

    sm.userdata.result_report = ""
    sm.userdata.temp_report = ""
    sm.userdata.selected_arm = "/right_arm"

    sm.userdata.question = ''
    sm.userdata.textinit = 'what can I do for you?'
    sm.userdata.textcome = 'Can someone come closer?'
    sm.userdata.timeinv = -1
    sm.userdata.timeout = 3
    sm.userdata.dicc = 'gpsr_c2'    
    
    sm.userdata.textcontinue = 'Do you have another request?'
    sm.userdata.timeoutcontinue = 2.5


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    with sm:
        smach.StateMachine.add('MOVE_TO_INIT_LOCATION',GoToPlace.getInstance(),
                transitions={'succeeded':'FIND_FACE'},
                remapping={'map_name':'map_name',
                           'place_name':'init_place'
                }
        )
        smach.StateMachine.add('FIND_FACE', FindFace.getInstance(),
                transitions = {'Face Detected':'COUNTING_TIMES',
                'Face Not Detected':'FIND_FACE'},
                remapping={'time':'timeinv',
                'textintro':'textcome'}
        )   
        smach.StateMachine.add('COUNTING_TIMES', CoutingTimes(),
                           transitions={'normal':'LISTEN_INSTRUCTIONS',
                                        'team':'LISTEN_INSTRUCTIONS',
                                        'keyboard':'GPSR_KEYBOARD'   }
        )
        smach.StateMachine.add('GPSR_KEYBOARD',GPSR_keyboard.getInstance(),
                transitions={'succeeded':'succeeded'}
        )   
        smach.StateMachine.add('LISTEN_INSTRUCTIONS', AskGPSR.getInstance(),
                transitions = {'succeeded':'PARSE'},
                remapping = {'question':'textinit',
                            'timeout':'timeout',
                            'dictionary':'dicc'}
        )
        smach.StateMachine.add('PARSE',ParseOrder.getInstance(),
                transitions={'succeeded':'EVALUATE_GPSR',
                             'aborted' : 'HEAR_FAIL'},
                remapping={'recognized_phrase':'answer'}
        )
        smach.StateMachine.add('EVALUATE_GPSR',EvaluateGPSR(),
               transitions={'succeeded':'CONFIRM',
                            'next':'EVALUATE_GPSR'}
        )
        smach.StateMachine.add('CONFIRM',AskForConfirmation.getInstance(),
                           transitions={'yes':'TALK_CONFIRM',
                                        'no':'RESET_INF'},
                           remapping={'text':'question',
                                    'timeout':'timeout'}
        )

        smach.StateMachine.add('TALK_CONFIRM',TalkState.getInstance('I will do it',2),
                            transitions={'succeeded':'ACTIONS'}
        )

        smach.StateMachine.add('HEAR_FAIL',TalkState.getInstance('I didnt hear you well', 2),
                            transitions={'succeeded':'COUNTING_TIMES'}
        )
        smach.StateMachine.add('RESET_INF',ResetInf(),
               transitions={'succeeded':'COUNTING_TIMES'}
        )
        smach.StateMachine.add('ACTIONS',Actions.getInstance(),
                transitions={'succeeded':'RESET_INF',
                              'unsucceeded':'GO_TO_PRE_EXIT'}
        )
        #TODO decir> he terminado con mis tareas ahora me puedo ir
        smach.StateMachine.add('GO_TO_PRE_EXIT', GoToPlace.getInstance(),
                transitions = {'succeeded':'EXIT'},
                remapping = {'place_name':'exit_pre',
                             'map_name':'map_name'}
        )

        smach.StateMachine.add('EXIT', GoToPlace.getInstance(),
                transitions = {'succeeded':'succeeded'},
                remapping = {'place_name':'exit_post',
                             'map_name':'map_name'}
        )

     #   initial_data = smach.UserData()
     #   initial_data.answer = 'offer a drink to the person at the door'
     #   sm.set_initial_state(['LISTEN_INSTRUCTIONS'],initial_data)

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('GPSR')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('gpsr', sm, '/GPSR_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
