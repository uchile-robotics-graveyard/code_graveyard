#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.speech import Talk
from bender_macros.speech import Recognize

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        rospy.sleep(0.1)
        return 'succeeded'

class AskQuestion(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['succeeded','aborted','preempted'],
                            input_keys=['question','timeout','dictionary']
                            )

    def execute(self, userdata):
        rospy.loginfo('Executing state AskQuestion... Talking')
        
        Talk.getInstance(userdata.question,userdata.timeout)
        return 'succeeded'


          
class SeparateInstructions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                 io_keys=['answer'])
        

    def execute(self, userdata):
        rospy.loginfo('Executing state SeparateInstructions')

        sentence = userdata.answer.split(" ")
        print sentence
        if len(sentence)<2:
            return 'preempted'

        output = ""
        for sen in sentence:
            parse=0
            if not output == "":
                output+=" "
            try:
                #name = '/ParseOrder/'+sen  
                #print name
                output+= self.load_yaml(sen) #rospy.get_param(name)
            except:
                output+=sen
           

        userdata.answer =  output
        print output
        return 'succeeded'

    def load_yaml(name):
        rospy.loginfo('Executing state FindActions')
        rospack = rospkg.RosPack()
        parser_path = rospack.get_path('bender_utils')+'/config/mapper/speechparser.yaml'

        try:
            with open(parser_path, 'r') as f:
                # load all documents
                parser_data = yaml.load(f)
                if parser_data is None:
                    raise yaml.YAMLError("Empty files not allowed")
        except yaml.YAMLError as e:
            rospy.logerr('Invalid YAML file: %s' % (str(e)))

        return parser_data[name][0]

      
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['question','timeout','dictionary'],
                            output_keys=['answer']
                            )


    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'ASK_QUESTION'}
                           )
        smach.StateMachine.add('ASK_QUESTION',AskQuestion(),
                           transitions={'succeeded':'RECOGNIZE'},
                           remapping={'question':'question',
                                    'timeout':'timeout'}
                           )
        smach.StateMachine.add('RECOGNIZE',Recognize.getInstance(),
                           transitions={'succeeded':'SeparateInstructions'},
                           remapping={'dictionary':'dictionary',
                                    'recognized_word':'answer'}
                           )
        smach.StateMachine.add('SeparateInstructions',SeparateInstructions(),
                           transitions={'succeeded':'succeeded',
                           'preempted':'RECOGNIZE'}
                           )

                           
    return sm


if __name__ == '__main__':

    rospy.init_node('AskRecognizeAndConfirm')

    sm = getInstance()
    ud = smach.UserData()

    ud.question = "What's your command"
    ud.timeout = 1.5
    ud.dictionary = 'gpsr_c1'
    # introspection server
    sis = smach_ros.IntrospectionServer('AskRecognizeAndConfirm', sm, '/ASK_RECOGNIZE_AND_CONFIRM_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
