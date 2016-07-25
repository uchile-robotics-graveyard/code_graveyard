#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
import math
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.nav import GoToPoseStamped

from bender_utils.ros import benpy

from geometry_msgs.msg import PoseStamped,Pose
from geometry_msgs.msg import Twist

class calculate_newpose(smach.State):

  def __init__(self):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
              input_keys=['distance'],
              io_keys=['NewPose'])

  def transform_pose(self,pose_in,frame):
      
      tf_req = TransformerRequest()
      tf_req.pose_in = pose_in
      tf_req.frame_out = frame
      transf_out = self.transform_client(tf_req)
      return transf_out.pose_out

  def execute(self, userdata):
    
    userdata.NewPose = PoseStamped()
    userdata.NewPose.position.y = distance

    userdata.NewPose = self.transform_pose(userdata.NewPose,"map")
    
    return 'succeeded'

def getInstance():

  sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
    input_keys=['distance'])

  sm.userdata.NewPose = PoseStamped

  sm.userdata.distance_back = -0.5


  with sm:
    smach.StateMachine.add('MOVE_REV', calculate_newpose(),
      transitions={'succeeded':'MOVEXBASE'}
      )
    smach.StateMachine.add('MOVEXBASE',MoveXbase.getInstance(),
      transitions={'succeeded':'GO_TO_POSE'},
      remapping={'distance':'distance_back'}
      )
      smach.StateMachine.add('GO_TO_POSE', GoToPoseStamped.getInstance(),
                   transitions = {'succeeded':'succeeded','aborted':'GO_TO_POSE'},
                   remapping = {'goal_pose':'NewPose'} )
      )


  #sm.set_initial_state(['MOVE_BACK'])
  return sm


# main
if __name__ == '__main__':

  rospy.init_node('move_y_base')

  sm = getInstance()

  ud = smach.UserData()

  # introspection server
  sis = smach_ros.IntrospectionServer('move_y_base', sm, '/MOVE_Y_BASE_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()