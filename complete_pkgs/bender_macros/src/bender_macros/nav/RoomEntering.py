#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import time

from geometry_msgs.msg import Twist
from bender_srvs.srv import synthesize

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class Entrance(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.entrance_duration = 3.5  # go ahead for 4 [s]
        
    def execute(self, userdata):
        
        talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        talk_client.wait_for_service()
        talk_client("I am entering the arena")
        
        pub = rospy.Publisher('/bender/nav/cmd_vel', Twist)

        order = Twist()
        
        # - - - - - avanzar - - - - -
        print "avanzando"
        order.linear.x = 0.3
        
        initial_time = time.time()
        while time.time() - initial_time < self.entrance_duration:
            pub.publish(order)

        # - - - - - frenar - - - - - -
        print "frenando"
        order.linear.x = 0.0
        pub.publish(order)
        rospy.sleep(0.1)
        
        return 'succeeded'