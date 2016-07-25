#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.speech import Recognize

def getInstance():
    
    dict = 'move_order'

    return Recognize.getInstance(dict)
  
# main
if __name__ == '__main__':

    rospy.init_node('nav_speech_order')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()

    # introspection server
    sis = smach_ros.IntrospectionServer('nav_speech_order', sm, '/NAV_SPEECH_ORDER_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
     
