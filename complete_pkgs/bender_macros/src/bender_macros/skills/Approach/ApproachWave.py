#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
from bender_srvs.srv import *
from bender_msgs.msg import *
# from bender_macros.nav import GoToPlace
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import RotateRobot

from std_srvs.srv import Empty
from bender_utils.ros import benpy
from bender_macros.speech import Talk
import roslaunch

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

   def __init__(self):
      smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

   def execute(self, userdata):
      rospy.loginfo('Executing state Setup')
      Talk.getInstance('I am looking for the waving person', 3)      
      
      return 'succeeded'


class launchP(smach.State):

    def __init__(self):
      smach.State.__init__(self, outcomes=['timeout','succeeded','aborted','preempted'],
                             output_keys=['sm_person_pose'])
      self.wave_on_client = rospy.ServiceProxy('/bender/ni/detect_wave', SearchPerson)#Nunca dejar con benpy
      self.wave_off_client = rospy.ServiceProxy('/bender/ni/shutdown_nite', IsOn)

    def execute(self, userdata):
      rospy.loginfo('Executing state launchP')

      init_time = time.time()

      package = 'bender_ni'
      executable = 'kinect_tracker'
      ns = '/bender/ni/'
      node = roslaunch.core.Node(package, executable, executable,ns)

      try:
         launch = roslaunch.scriptapi.ROSLaunch()
         launch.start()
         process = launch.launch(node)
      except:
         return 'aborted'

      if not process.is_alive():
         return 'aborted'

      rospy.sleep(0.5)
      self.wave_on_client.wait_for_service()
      try:
         resp = self.wave_on_client()
      except:
        process.stop()
        return 'aborted'
      

      print "service"
      intime = True
      while not resp.found and intime:
        resp = self.wave_on_client()
        if time.time()-init_time > 15:
          intime = False
        rospy.sleep(0.5)

      print "service off"
      while not self.wave_off_client():
         rospy.sleep(0.5)

      process.stop()

      if intime:
        userdata.sm_person_pose = resp.person
        print resp
        return 'succeeded'

      return 'timeout'


class TransformPose(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             io_keys=['sm_person_pose'])
        self.transform_client = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)

    def execute(self, userdata):
        rospy.loginfo('Executing state TransformPose')

        print userdata.sm_person_pose
        tf_req = TransformerRequest()
        tf_req.pose_in = userdata.sm_person_pose
        tf_req.frame_out = "map"
        transf_out = self.transform_client(tf_req)
        userdata.sm_person_pose = transf_out.pose_out

        print userdata.sm_person_pose
        return 'succeeded'

        
class CheckPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                        input_keys=['sm_person_pose'])

        self.room_detection_client = benpy.ServiceProxy('/bender/nav/map_analyzer/check_point_inside_map', ValidPoint)

    def execute(self, userdata):
        rospy.loginfo('Executing state CompareRoom')

        req = ValidPointRequest()
        req.point.x = userdata.sm_person_pose.pose.position.x
        req.point.y = userdata.sm_person_pose.pose.position.y
        req.point.z = userdata.sm_person_pose.pose.position.z
        req.frame_id = userdata.sm_person_pose.header.frame_id

        resp = self.room_detection_client (req)

        if resp.is_valid:
          return 'succeeded'
        return 'aborted'      




def getInstance():

      sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])

      sm.userdata.map_name = 'map.sem_map'
      sm.userdata.sm_person_pose = PoseStamped()

      sm.userdata.angle = 50

      with sm:

            smach.StateMachine.add('SETUP', Setup(),
                transitions={'succeeded':'Launch'}
            )
            smach.StateMachine.add('Launch',launchP(),
                transitions={'succeeded':'CHECKPOSE',
                           'timeout':'TURN_AROUND',
                          'aborted':'Launch'}
            )
            smach.StateMachine.add('TURN_AROUND',RotateRobot.getInstance(),
                transitions = {'succeeded':'Launch',
                               'aborted':'Launch'},
                remapping   = {'angle':'angle'}
            )
            smach.StateMachine.add('CHECKPOSE',CheckPose(),
                transitions={'succeeded':'TRANSFORM',
                               'aborted':'Launch'}
            )
            smach.StateMachine.add('TRANSFORM',TransformPose(),
                transitions={'succeeded':'APPROACH_TO_PERSON'}
            )
            smach.StateMachine.add('APPROACH_TO_PERSON', ApproachToPoseStamped.getInstance(),
                transitions = {'succeeded':'LOOK_PERSON','aborted':'Launch'},
                remapping = {'goal_pose':'sm_person_pose'} 
            )
            smach.StateMachine.add('LOOK_PERSON', LookToPoseStamped.getInstance(),
                transitions = {'succeeded':'succeeded','aborted':'LOOK_PERSON'},
                remapping = {'goal_pose':'sm_person_pose'}
            )
      return sm


# main
if __name__ == '__main__':

      rospy.init_node('launchProcess')

      sm = getInstance()

      # introspection server
      sis = smach_ros.IntrospectionServer('launchProcess', sm, '/launchProcessSM')
      sis.start()
      outcome = sm.execute()
      sis.stop()