#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros

from bender_macros.arm import ArmStates
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import PoseStamped
#from bender_core import benpy

def getInstance():

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])

  sm.userdata.trayectory_name_before = ['premanip_1']
  sm.userdata.trayectory_name_dance_pollo = ['pollodance_1','pollodace_2','pollodance_1']
  sm.userdata.trayectory_name_dance_noanoa = ['noanoadance_1','noanoadance_2','noanoadance_3','noanoadance_4','noanoadance_5','noanoadance_1']
  sm.userdata.trayectory_name_after_close = ['premanip_2','premanip_1','home']

  sm.userdata.trayectory_home = 'home'
  sm.userdata.effort = 500

  R = Limb('r')
  L = Limb('l')

  with sm:

    smach.StateMachine.add('MOVE_NAMED_BEFOREL', ArmStates.setPositionNamed(L),
               transitions={'succeeded':'MOVE_NAMED_BEFORER','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before'}
               )
    smach.StateMachine.add('MOVE_NAMED_BEFORER', ArmStates.setPositionNamed(R),
               transitions={'succeeded':'DANCE_POLLO_R','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before'}
               )
    smach.StateMachine.add('DANCE_POLLO_R', ArmStates.setPositionNamed(R),
               transitions={'succeeded':'DANCE_POLLO_L','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_dance_pollo'}
               )
    smach.StateMachine.add('DANCE_POLLO_L', ArmStates.setPositionNamed(L),
               transitions={'succeeded':'GO_HOMEL','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_dance_pollo'}
               )

    smach.StateMachine.add('GO_HOMEL', ArmStates.setPositionNamed(L),
               transitions={'succeeded':'GO_HOMER','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_home'}
               )
    smach.StateMachine.add('GO_HOMER', ArmStates.setPositionNamed(R),
               transitions={'succeeded':'succeeded','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_home'}
               )
  sm.set_initial_state(['MOVE_NAMED_BEFOREL'])
  return sm

# main
if __name__ == '__main__':

  rospy.init_node('DanceCartesian')

  sm = getInstance()
  ud = smach.UserData()
  

  # introspection server
  sis = smach_ros.IntrospectionServer('DanceCartesian', sm, '/DANCE_CARTESIAN_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()