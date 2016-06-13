#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
import math
from bender_srvs.srv import *
from bender_msgs.msg import *

from bender_macros.nav import ApproachToPlane

from bender_macros.arm import ArmStates
from bender_utils.ros import benpy
from bender_arm_control.arm_commander import Limb

from bender_macros.nav import MoveBaseX

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from bender_macros.head import MoveAsus

def getInstance():

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
    input_keys=['place_pose','lr_arm'])

  sm.userdata.trayectory_name_before = ['premanip_1']
  sm.userdata.trayectory_name_lift = ['premanip_2']
  sm.userdata.trayectory_name_after_close = ['premanip_2','premanip_1','home']

  sm.userdata.effort = 600
  sm.userdata.approx_ik = True
  sm.userdata.distance_back = -0.30
  sm.userdata.distance_fwd = 0.3

  p = PoseStamped()
  p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
  p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.55, -0.15, 1.1
  p.pose.orientation = Limb.simple_orientation

  sm.userdata.place_shelf = p

  sm.userdata.distance_fwd_shelf2 = 0.55
  sm.userdata.distance_fwd_shelf3 = 0.65

  arms = {'r':Limb('r'),'l':Limb('l')}
  
  sm.userdata.deg_anglein = 50

  with sm:
    # smach.StateMachine.add('LIFT_BEFORE',ArmStates.setPositionNamed(arms),
    #   transitions={'succeeded':'succeeded','aborted':'aborted'},
    #   remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
    #   )
    smach.StateMachine.add('MOVEASUS_ini',MoveAsus.getInstance(),
        transitions={'succeeded':'MOVE_REV'},
        remapping={'deg_angle':'deg_anglein'}
        )
    smach.StateMachine.add('MOVE_REV', MoveBaseX.getInstance(),
      transitions={'succeeded':'LIFT_GRIPPER'},
      remapping={'d':'distance_back'}
      )
    smach.StateMachine.add('LIFT_GRIPPER', ArmStates.setPositionNamed(arms,blind=True,init='carry',goal='shelf_1'),
      transitions={'succeeded':'MOVE_FWD','aborted':'aborted'},
      remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
      )
    # smach.StateMachine.add('LIFT_GRIPPER',ArmStates.setPositionNamed(arms),
    #   transitions={'succeeded':'PLACE','aborted':'aborted'},
    #   remapping={'trayectory_name':'trayectory_name_lift','lr_arm':'lr_arm'}
    #   )
    smach.StateMachine.add('PLACE', ArmStates.positionEfector_capmap(arms),
      transitions={'succeeded':'MOVE_FWD','aborted':'aborted'},
      remapping={'pose':'place_shelf','heigh':'heigh','radius':'radius','lr_arm':'lr_arm'}
      )
    # smach.StateMachine.add('MOVE_FWD', MoveBaseX.getInstance(),
    #   transitions={'succeeded':'OPEN_GRIP'},
    #   remapping={'d':'distance_fwd'}
    #   )
    smach.StateMachine.add('MOVE_FWD', ApproachToPlane.getInstance(),
      transitions={'succeeded':'OPEN_GRIP'},
      remapping={'distance':'distance_fwd_shelf3'}
      )
    # smach.StateMachine.add('MOVE_FWD', ApproachToPlane.getInstance(),
    #   transitions={'succeeded':'OPEN_GRIP'},
    #   remapping={'distance':'distance_fwd_shelf3'}
    #   )
    smach.StateMachine.add('OPEN_GRIP', ArmStates.openGripper(arms),
      transitions={'succeeded':'MOVE_REV2','aborted':'aborted'},
      ) 
    smach.StateMachine.add('MOVE_REV2', MoveBaseX.getInstance(),
      transitions={'succeeded':'CLOSE_GRIP'},
      remapping={'d':'distance_back'}
      )
    smach.StateMachine.add('CLOSE_GRIP', ArmStates.closeGripper(arms),
      transitions={'notgrabbing':'MOVE_NAMED_AFTER_CLOSE','stalled':'MOVE_NAMED_AFTER_CLOSE','aborted':'aborted'},
      ) 
    smach.StateMachine.add('MOVE_NAMED_AFTER_CLOSE', ArmStates.setPositionNamed(arms),
      transitions={'succeeded':'Approach_FWD','aborted':'aborted'},
      remapping={'trayectory_name':'trayectory_name_after_close','lr_arm':'lr_arm'}
      )
    smach.StateMachine.add('MOVE_FWD2', MoveBaseX.getInstance(),
      transitions={'succeeded':'Approach_FWD'},
      remapping={'d':'distance_fwd'}
      )
    smach.StateMachine.add('Approach_FWD', ApproachToPlane.getInstance(),
      transitions={'succeeded':'succeeded'},
      remapping={'distance':'distance_fwd_shelf2'}
      )


  #sm.set_initial_state(['MOVE_BACK'])
  return sm


# main
if __name__ == '__main__':

  rospy.init_node('place_shelf_sm')

  sm = getInstance()

  ud = smach.UserData()
  p = PoseStamped()
  p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
  p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.5, -0.15, 1.1
  p.pose.orientation = Limb.simple_orientation

  ud.place_pose = p
  ud.lr_arm = 'l'

  # introspection server
  sis = smach_ros.IntrospectionServer('place_shelf_sm', sm, '/PLACE_SHELF_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()
