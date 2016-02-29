#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from bender_srvs.srv import *
from bender_msgs.msg import *
import roslaunch


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        rospy.sleep(1)
        return 'succeeded'

# define state Position
class Launch(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','canceled'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Position')
        
        package = 'bender_ni'
        executable = 'kinect_tracker'
        node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        print process.is_alive()

        return 'succeeded'



def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted','canceled'])

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'LAUNCH'})

        smach.StateMachine.add('LAUNCH',Launch(),
                           transitions={'succeeded':'succeeded'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('launchExample')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('launch_example', sm, '/LAUNCH_EXAMPLE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()