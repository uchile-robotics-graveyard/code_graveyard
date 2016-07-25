#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import GoalFromPlace

def getInstance():
    
    return GoalFromPlace.getMachineInstance(
                LookToPoseStamped.getInstance()
    )
  
# main
if __name__ == '__main__':

    rospy.init_node('look_to_place')

    sm = getInstance()

    # generate dummy userdata
    ud = smach.UserData()
    ud.place_name = 'tv'
    ud.map_name = 'map.sem_map'

    # introspection server
    sis = smach_ros.IntrospectionServer('look_to_place', sm, '/LOOK_TO_PLACE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 
    