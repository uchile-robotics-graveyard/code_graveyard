#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.speech import Recognize

def getInstance():
    
    dict = 'gpsr'

    return Recognize.getInstance(dict)
  
# main
if __name__ == '__main__':

    rospy.init_node('gpsr_order')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()

    # introspection server
    sis = smach_ros.IntrospectionServer('gpsr_order', sm, '/GPSR_ORDER_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
     
