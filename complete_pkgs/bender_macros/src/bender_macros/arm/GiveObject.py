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
#from bender_utils.ros import benpy

def getInstance():

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],input_keys=['lr_arm'])

  sm.userdata.trayectory_name_before = ['premanip_1','premanip_2']
  sm.userdata.trayectory_name_after_grasp = ['premanip_1']
  sm.userdata.trayectory_name_after_close = ['premanip_2','premanip_1','home']

  sm.userdata.trayectory_home = 'home'
  sm.userdata.effort = 100

  sm.userdata.text = "Take the object, please"
  sm.userdata.timeout = 3

  #L = {'l':Limb('l'),'r':Limb('l')}
  L = {'l':Limb('l'),'r':Limb('r')}

  with sm:
    smach.StateMachine.add('MOVE_PRE_1', ArmStates.setPositionNamed(L,blind=True,init='home',goal='pre_1'),
               transitions={'succeeded':'MOVE_PRE_2','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('MOVE_PRE_2', ArmStates.setPositionNamed(L,blind=True,init='pre_1',goal='give'),
               transitions={'succeeded':'TALK','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('TALK', TalkState.getInstance(),
               transitions={'succeeded':'OPEN_GRIP','aborted':'aborted'},
               remapping={'text':'text','timeout':'timeout'}
               )
    smach.StateMachine.add('OPEN_GRIP', ArmStates.openGripper(L),
               transitions={'succeeded':'CLOSE_GRIP','aborted':'aborted'},
               )
    smach.StateMachine.add('CLOSE_GRIP', ArmStates.closeGripper(L),
               transitions={'notgrabbing':'MOVE_NAMED_AFTER_CLOSE','stalled':'MOVE_NAMED_AFTER_CLOSE','aborted':'aborted'},
               remapping={'effort':'effort','lr_arm':'lr_arm'}
               )
    smach.StateMachine.add('MOVE_NAMED_AFTER_CLOSE', ArmStates.setPositionNamed(L,blind=False,init='pre_2',goal='pre_1'),
               transitions={'succeeded':'GO_HOME','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_name_after_grasp','lr_arm':'lr_arm'}
               )
    # smach.StateMachine.add('MOVE_NAMED_AFTER_CLOSE', ArmStates.setPositionNamed(L,blind=True,init='pre_2',goal='pre_1'),
    #            transitions={'succeeded':'GO_HOME','aborted':'aborted'},
    #            remapping={'trayectory_name':'trayectory_name_after_grasp','lr_arm':'lr_arm'}
    #            )
    smach.StateMachine.add('GO_HOME', ArmStates.setPositionNamed(L,blind=True,init='pre_1',goal='home'),
               transitions={'succeeded':'succeeded','aborted':'aborted'},
               remapping={'trayectory_name':'trayectory_home','lr_arm':'lr_arm'}
               )
  #sm.set_initial_state(['MOVE_PRE_1'])
  return sm

# main
if __name__ == '__main__':

  rospy.init_node('GraspCartesian')

  sm = getInstance()
  ud = smach.UserData()
  
  # genera userdata de prueba
  p = PoseStamped()
  p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
  p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.5, -0.25, 0.9
  p.pose.orientation = Limb.simple_orientation

  ud.lr_arm = 'l'

  # introspection server
  sis = smach_ros.IntrospectionServer('GraspCartesian', sm, '/GRASP_CARTESIAN_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()
