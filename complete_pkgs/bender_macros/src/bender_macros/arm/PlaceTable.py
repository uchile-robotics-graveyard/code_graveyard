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
from bender_macros.speech import Talk
#from bender_utils.ros import benpy

class Apologize(smach.State):

  def __init__(self):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
               output_keys=['report'])

  def execute(self, userdata):
    rospy.loginfo('Executing state Apologize')

    Talk.getInstance('I am sorry, I could not manipulate the object.', 4)

    return 'succeeded'

def getInstance():

  '''
  Esta maquna de estados:
  - levanta el brazo 'lr_arm' con un objeto tomado a las posiciones 'pre_1' y 'pre_2'
  - se mueve a una 'pose' con el objeto tomado (planificando)
  - suelta el objeto y se devuelve a 'home'

  '''

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],input_keys=['lr_arm','pose','height','radius'])

  sm.userdata.trayectory_name_after_place = ['premanip_2']
  sm.userdata.trayectory_name_after_grasp = ['premanip_2']
  sm.userdata.effort = 0.1
  sm.userdata.approx_ik = False

  arms = {'l':Limb('l'),'r':Limb('r')}

  with sm:
    smach.StateMachine.add('MOVE_PRE_1', ArmStates.setPositionNamed(arms,blind=True,init='home',goal='pre_1'),
                transitions={'succeeded':'MOVE_PRE_2','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )  
    smach.StateMachine.add('MOVE_PRE_2', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='pre_2'),
                transitions={'succeeded':'PLACE_POSE','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
    smach.StateMachine.add('PLACE_POSE', ArmStates.positionEfector_capmap(arms),
                transitions={'succeeded':'OPEN_GRIP','aborted':'APOLOGIZE'},
                remapping={'posestamped':'pose','lr_arm':'lr_arm','height':'height','radius':'radius'}
                )
    smach.StateMachine.add('APOLOGIZE',Apologize(),
                transitions={'succeeded':'MOVE_NAMED_AFTER_PLACE'})
    smach.StateMachine.add('OPEN_GRIP', ArmStates.openGripper(arms),
                transitions={'succeeded':'MOVE_NAMED_AFTER_PLACE','aborted':'aborted'},
                )
    smach.StateMachine.add('MOVE_NAMED_AFTER_PLACE', ArmStates.setPositionNamed(arms,blind=False),
                transitions={'succeeded':'CLOSE_GRIP','aborted':'aborted'},
                remapping={'trayectory_name':'trayectory_name_after_place','lr_arm':'lr_arm'}
                ) 
    smach.StateMachine.add('CLOSE_GRIP', ArmStates.closeGripper(arms),
                transitions={'notgrabbing':'MOVE_PRE_2_PRE_1','stalled':'MOVE_PRE_2_PRE_1','aborted':'aborted'},
                remapping={'effort':'effort','lr_arm':'lr_arm'}
                ) 
    smach.StateMachine.add('MOVE_PRE_2_PRE_1', ArmStates.setPositionNamed(arms,blind=True,init='pre_2',goal='pre_1'),
                transitions={'succeeded':'GO_HOME','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
    smach.StateMachine.add('GO_HOME', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='home'),
                transitions={'succeeded':'succeeded','aborted':'aborted'},
                remapping={'lr_arm':'lr_arm'}
                )
  #sm.set_initial_state(['MOVE_PRE_1'])
  return sm

# main
if __name__ == '__main__':

  rospy.init_node('PlaceTable')

  sm = getInstance()
  ud = smach.UserData()
  
  # genera userdata de prueba
  p = PoseStamped()
  p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
  p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.5, 0.0, 0.9
  p.pose.orientation = Limb.simple_orientation

  ud.lr_arm = 'l'
  ud.pose = p
  ud.height = 0.1
  ud.radius = 0.04

  # introspection server
  sis = smach_ros.IntrospectionServer('PlaceTable', sm, '/PLACE_TABLE_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()
