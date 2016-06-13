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

from bender_macros.speech import AskRecognizeAndConfirm     
from bender_macros import BringObject



class Appologize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Introduce')
    
        Talk.getInstance('Sorry, buy I cant find the object', 3)

        return 'succeeded'  


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


      
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                   input_keys=['textoffer','dicc'])

    sm.userdata.map_name = 'map.sem_map'
    sm.userdata.timeout = 3


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    with sm:

        smach.StateMachine.add('OFFER',AskRecognizeAndConfirm.getInstance(),
               transitions={'succeeded':'BRING'},
                remapping = {'question':'textoffer',
                            'timeout':'timeout',
                            'dictionary':'dicc'}
                            )

        smach.StateMachine.add('BRING',BringObject.getInstance(),
               transitions={'succeeded':'succeeded',
                            'not_found':'APOLOGIZE',
                             'not_grabbed':'APOLOGIZE' },
                remapping = {'object_name':'answer'}
                            )
        smach.StateMachine.add('APOLOGIZE',Appologize(),
               transitions={'succeeded':'succeeded'}
                            )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('OFFER')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('offer', sm, '/OFFER_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()