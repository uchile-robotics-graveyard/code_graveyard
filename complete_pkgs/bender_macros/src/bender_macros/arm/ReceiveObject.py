#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros

from bender_macros.arm import ArmStates
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import PoseStamped
from bender_macros.speech import TalkState


def getInstance():

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                input_keys=['lr_arm'])

  sm.userdata.trayectory_name_before = ['premanip_1','premanip_2']
  sm.userdata.trayectory_name_after_grasp = ['premanip_2','premanip_1']

  sm.userdata.trayectory_home = 'home'
  sm.userdata.effort = 500

  sm.userdata.text = 'Please put the object in my hand'
  sm.userdata.timeout = 4
  sm.userdata.text2 = 'thank you'
  sm.userdata.timeout2 = 2

  L = {'l':Limb('l'),'r':Limb('l')}

  with sm:

    smach.StateMachine.add('MOVE_PRE_1', ArmStates.setPositionNamed(L,blind=True,init='home',goal='pre_1'),
               transitions={'succeeded':'MOVE_PRE_2','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('MOVE_PRE_2', ArmStates.setPositionNamed(L,blind=True,init='pre_1',goal='pre_2'),
               transitions={'succeeded':'OPEN_GRIP','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('OPEN_GRIP', ArmStates.openGripper(L),
               transitions={'succeeded':'TALK','aborted':'aborted'},
               )
    smach.StateMachine.add('TALK', TalkState.getInstance(),
               transitions={'succeeded':'CLOSE_GRIP','aborted':'aborted'},
               remapping={'text':'text','timeout':'timeout'}
               )
    smach.StateMachine.add('CLOSE_GRIP', ArmStates.closeGripper(L),
               transitions={'notgrabbing':'OPEN_GRIP','stalled':'TALK2','aborted':'aborted'},
               remapping={'effort':'effort'}
               )
    smach.StateMachine.add('TALK2', TalkState.getInstance(),
               transitions={'succeeded':'MOVE_NAMED_AFTER_GRASP','aborted':'aborted'},
               remapping={'text':'text2','timeout':'timeout2'}
               )
    smach.StateMachine.add('MOVE_NAMED_AFTER_GRASP', ArmStates.setPositionNamed(L,blind=True,init='pre_2',goal='pre_1'),
               transitions={'succeeded':'GO_HOME','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_after_grasp','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('GO_HOME', ArmStates.setPositionNamed(L,blind=True,init='pre_1',goal='home'),
               transitions={'succeeded':'succeeded','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_home','lr_arm':'lr_arm'}
               )

  sm.set_initial_state(['MOVE_PRE_1'])
  return sm

# main
if __name__ == '__main__':

  rospy.init_node('receive_object')

  sm = getInstance()
  ud = smach.UserData()
  
  ud.lr_arm = 'l'

  # introspection server
  sis = smach_ros.IntrospectionServer('receive_object', sm, '/RECEIVE_OBJECT_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()