#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.speech import Recognize

def getInstance():
    
    dict = 'name'

    return Recognize.getInstance(dict)
  
# main
if __name__ == '__main__':

    rospy.init_node('ask_for_names')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()

    # introspection server
    sis = smach_ros.IntrospectionServer('ask_for_names', sm, '/ASK_FOR_NAMES_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
     
