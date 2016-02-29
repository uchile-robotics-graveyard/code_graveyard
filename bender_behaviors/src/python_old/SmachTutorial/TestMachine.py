#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

import CoreMachine
import SubMachine

# main
if __name__ == '__main__':


    rospy.init_node('behavior_core')

    sub_machine = SubMachine.getInstance()
    
    core = CoreMachine.CoreMachine('SUB_MACHINE',sub_machine)
    core.launchIntrospectionServer()

