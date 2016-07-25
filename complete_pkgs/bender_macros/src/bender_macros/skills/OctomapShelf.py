#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.head import MoveAsus
from bender_macros.nav import RotateRobot
from bender_macros.speech import Talk
from bender_utils.ros import benpy
# from bender_macros.skills import Octomap
from bender_maps import OctomapManager

#from std_srvs.srv import Empty

#import roslaunch

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


class InitOctomap(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.oct = OctomapManager()

    def execute(self, userdata):
        rospy.loginfo('Executing state PubShelf')

        self.oct.start()
        self.oct.clear()
        rospy.sleep(2)
        self.oct.stop()

        return 'succeeded'


class SelectRGBDAngle(smach.State):

   def __init__(self, rgbd_angles):
      smach.State.__init__(self, 
                           outcomes=['finish','succeeded','aborted','preempted'],
                           output_keys=['selected'])
      # angles
      self.angles = rgbd_angles
      self.ang_id = -1

   def execute(self, userdata):
      rospy.loginfo('Executing state SelectRGBDAngle')

      self.ang_id+=1

      if self.ang_id >= len(self.angles):
         return 'finish'

      nang = self.ang_id%len(self.angles)
      userdata.selected = self.angles[nang]
      rospy.loginfo('setting rgbd angle> ' + str(self.angles[nang]))

      return 'succeeded'



class CheckOctomap(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.oct = OctomapManager()

    def execute(self, userdata):
        rospy.loginfo('Executing state PubShelf')

        self.oct.start()
        rospy.sleep(1)
        self.oct.stop()
        
        return 'succeeded'

    
def getInstance():

      sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])


      # rgbd
      rgbd_angles = [ 10, 20, 30,40, 50, 60]
      sm.userdata.current_rgbd_angle = 20
      sm.userdata.ang_final = 40
      sm.userdata.ang_prepare = 40


      with sm:
            smach.StateMachine.add('ClearOctomap', InitOctomap(),
                transitions={'succeeded':'SELECT_RGBD_ANGLE'}
            )
            ## scan looking for a shelf
            smach.StateMachine.add('SELECT_RGBD_ANGLE', SelectRGBDAngle(rgbd_angles),
                transitions={'succeeded':'MOVE_RGBD',
                             'finish':'succeeded'},
                remapping={'selected':'current_rgbd_angle'}
            )

            smach.StateMachine.add('MOVE_RGBD', MoveAsus.getInstance(),
                transitions={'succeeded':'CHECK_OCTOMAP'},
                remapping={'deg_angle':'current_rgbd_angle'}
            )            

            smach.StateMachine.add('CHECK_OCTOMAP', CheckOctomap(),
                transitions={'succeeded':'SELECT_RGBD_ANGLE'}
            )
            
      return sm


# main
if __name__ == '__main__':

      rospy.init_node('octomap_shelf')

      sm = getInstance()

      # introspection server
      sis = smach_ros.IntrospectionServer('octomap_shelf', sm, '/OCTOMAPSHELFSM')
      sis.start()
      outcome = sm.execute()
      sis.stop()
