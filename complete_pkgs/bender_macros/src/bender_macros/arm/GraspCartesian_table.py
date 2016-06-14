#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros

from bender_macros.arm import ArmStates
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import PoseStamped
from bender_macros.speech import Talk
from bender_msgs.msg import CylindricalObject#, CubeObject
from std_srvs.srv import Empty
from bender_utils.ros import benpy
# Octomap
from bender_arm_planning.srv import ManageOctomap, ManageOctomapRequest
from bender_arm_planning.msg import OctomapOptions

# class GraspPose(smach.State):

#     def __init__(self,limb):
#         smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
#               input_keys = ['pose','height','radius'])
#         self.arms = limb

#     def execute(self, userdata):
#         self.limb = self.arms[userdata.lr_arm]
#         rospy.loginfo('Executing state GraspPose')

#         L = self.limb
#         obj = CylindricalObject()
#         # if userdata.type == "CUBE":
#         #   obj = CubeObject()

#         obj.pose = userdata.pose.pose

#         obj.height = userdata.height
#         obj.radius = userdata.radius

#         posible_grasp = L.arm.grasp(obj)
#         if len(posible_grasp.costs)==0:
#           return 'preempted'

#         rospy.loginfo("Cost: " + str(posible_grasp.costs[0]))
#         print posible_grasp.joint_names
#         L.arm.move_to_point(posible_grasp.points[0], posible_grasp.joint_names, interval=2.0)

#         rospy.sleep(3.5)
#         return 'succeeded'

class Apologize(smach.State):

  def __init__(self):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
               output_keys=['report'])

  def execute(self, userdata):
    rospy.loginfo('Executing state Apologize')

    Talk.getInstance('I am sorry, I could not manipulate the object.', 4)

    return 'succeeded'

def selectArm(posestamped):

  if posestamped.pose.position.y > 0:
    rospy.loginfo("I will use my LEFT arm")
    return 'l'
  elif posestamped.pose.position.y <= 0:
    rospy.loginfo("I will use my RIGHT arm")
    return 'r'

class Setup(smach.State):

  def __init__(self,arms):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
               io_keys=['pose'],
               output_keys=['lr_arm'])
    self.arms=arms

  def execute(self, userdata):
    rospy.loginfo('Executing state Setup')
    ar = selectArm(userdata.pose)
    self.arms[ar].gripper.close()
    if ar=='r':
      userdata.pose.pose.position.z+=0.04
      
    userdata.lr_arm = ar
    return 'succeeded'

def getInstance():

  sm = smach.StateMachine(outcomes=['notgrabbing','succeeded','preempted','aborted'],
              input_keys = ['pose','height','radius'],
               output_keys=['lr_arm'])

  sm.userdata.trayectory_name_before = ['home','premanip_1','premanip_2']
  sm.userdata.trayectory_name_after_grasp = ['premanip_2','premanip_1']
  sm.userdata.trayectory_name_after_close = ['premanip_2']
  sm.userdata.trayectory_name_carry = ['carrypos']

  sm.userdata.trayectory_home = 'home'
  sm.userdata.effort = 0.3

  arms = {'l':Limb('l'),'r':Limb('l')}

  with sm:
      smach.StateMachine.add('SETUP',Setup(arms),
                          transitions={'succeeded':'MOVE_HOME_PRE1'},
                            remapping={'lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_HOME_PRE1', ArmStates.setPositionNamed(arms,blind=True,init='home',goal='pre_1'),
                           transitions={'succeeded':'MOVE_PRE1_PRE2','aborted':'aborted'},
                            remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_PRE1_PRE2', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='pre_2'),
                           transitions={'succeeded':'GRASP_POSE','aborted':'aborted'},
                           remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('GRASP_POSE', ArmStates.positionObjectAndGrasp_capmap(arms),
                            transitions={'succeeded':'CLOSE_GRIP','aborted':'APOLOGIZE'},
                            remapping={'posestamped':'pose','height':'height','radius':'radius','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('CLOSE_GRIP', ArmStates.closeGripper(arms),
                           transitions={'notgrabbing':'MOVE_NAMED_GRASP','stalled':'MOVE_NAMED_GRASP','aborted':'MOVE_NAMED_NOT_GRABBED'},
                            remapping={'effort':'effort','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('APOLOGIZE',Apologize(),
                           transitions={'succeeded':'MOVE_NAMED_NOT_GRABBED'})

      smach.StateMachine.add('CARRY_POSITION',ArmStates.setPositionNamed(arms,blind=False),
                            transitions={'succeeded':'succeeded'},
                            remapping={'trayectory_name':'trayectory_name_carry','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_PRE2_PRE1', ArmStates.setPositionNamed(arms,blind=True,init='pre_2',goal='pre_1'),
                            transitions={'succeeded':'MOVE_PRE1_HOME','aborted':'aborted'},
                            remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_PRE1_HOME', ArmStates.setPositionNamed(arms,blind=True,init='pre_1',goal='home'),
                            transitions={'succeeded':'notgrabbing','aborted':'aborted'},
                            remapping={'trayectory_name':'trayectory_name_before','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_NAMED_NOT_GRABBED', ArmStates.setPositionNamed(arms),
                            transitions={'succeeded':'MOVE_PRE2_PRE1','aborted':'aborted'},
                            remapping={'trayectory_name':'trayectory_name_after_close','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('MOVE_NAMED_GRASP', ArmStates.setPositionNamed(arms,blind=False),
                            transitions={'succeeded':'GO_HOME','aborted':'aborted'},
                            remapping={'trayectory_name':'trayectory_name_after_grasp','lr_arm':'lr_arm'}
                            )
      smach.StateMachine.add('GO_HOME', ArmStates.setPositionNamed(arms),
                           transitions={'succeeded':'succeeded','aborted':'aborted'},
                           remapping={'trayectory_name':'trayectory_home','lr_arm':'lr_arm'}
                           )

  #sm.set_initial_state(['SETUP'])
  return sm

# main
if __name__ == '__main__':

  rospy.init_node('GraspCartesian')

  sm = getInstance()
  ud = smach.UserData()
  
  # genera userdata de prueba
  p = PoseStamped()
  p.header.frame_id = 'bender/base_link' #'bender/sensors/rgbd_head_rgb_optical_frame'
  p.pose.position.x,p.pose.position.y,p.pose.position.z = 0.58, 0.15, 0.85
  p.pose.orientation = Limb.simple_orientation

  ud.pose = p
  ud.height = 0.1
  ud.radius = 0.04

  #introspection server
  sis = smach_ros.IntrospectionServer('GraspCartesian', sm, '/GRASP_CARTESIAN_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()
