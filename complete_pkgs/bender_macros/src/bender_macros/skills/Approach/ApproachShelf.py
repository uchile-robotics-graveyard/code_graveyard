#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.nav import GoToPoseStamped
from bender_macros.nav import ApproachToPlane
from bender_macros.nav import ApproachToPoseStamped
from bender_macros.nav import LookToPoseStamped
from bender_macros.head import MoveAsus
from bender_macros.nav import RotateRobot
from bender_macros.speech import Talk
from bender_utils.ros import benpy
from bender_macros.head import FaceOrder


from bender_macros.nav import MoveBaseX

#from std_srvs.srv import Empty

#import roslaunch

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

   def __init__(self):
      smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
      self.shelf_enable_client = benpy.ServiceProxy('/bender/pcl/shelf_detector/enable', Onoff)

   def execute(self, userdata):
      rospy.loginfo('Executing state Setup')
      Talk.getInstance('Now, i will look for a shelf', 2)
      self.shelf_enable_client(True)

      return 'succeeded'


class SearchShelf(smach.State):

   def __init__(self):
      smach.State.__init__(self, outcomes=['fail','succeeded','aborted','preempted'],
                             output_keys=['sm_shelf_pose'])
      
      self.shelf_enable_client = benpy.ServiceProxy('/bender/pcl/shelf_detector/enable', Onoff)
      self.shelf_detect_client = benpy.ServiceProxy('/bender/pcl/shelf_detector/goal_shelf', PoseStamped)

   def execute(self, userdata):
      rospy.loginfo('Executing state SearchShelf')
      rospy.sleep(1)

      req = PoseStampedRequest()
      resp = self.shelf_detect_client(req)
      
      
      if resp.pose_out.pose.position.x == 0 and resp.pose_out.pose.position.y == 0 and resp.pose_out.pose.position.z == 0:
          return 'fail'
      
      print resp.pose_out  
      userdata.sm_shelf_pose = resp.pose_out
      Talk.getInstance("I found the shelf, i am going",3)
      self.shelf_enable_client(False)

      return 'succeeded'


class SelectRGBDAngle(smach.State):

   def __init__(self, rgbd_angles):
      smach.State.__init__(self, 
                           outcomes=['yaw','succeeded','aborted','preempted'],
                           output_keys=['selected'])
      # angles
      self.angles = rgbd_angles
      self.ang_id = -1

   def execute(self, userdata):
      rospy.loginfo('Executing state SelectRGBDAngle')

      self.ang_id+=1

      if self.ang_id >= len(self.angles):
        self.angles.sort(reverse=True)
        self.ang_id = -1
        Talk.getInstance('I cant find it, I will rotate to keep looking', 3)

        return 'yaw'

      nang = self.ang_id%len(self.angles)
      userdata.selected = self.angles[nang]
      rospy.loginfo('setting rgbd angle> ' + str(self.angles[nang]))

      return 'succeeded'


class SelectRobotAngle(smach.State):

   def __init__(self, robot_angles):
      smach.State.__init__(self,
                           outcomes=['succeeded','aborted','preempted'],
                           io_keys=['selected'])

      # angles
      self.angles = robot_angles
      self.ang_id = -1

   def execute(self, userdata):
      rospy.loginfo('Executing state SelectRobotAngle')

      self.ang_id+=1

      nang = self.ang_id%len(self.angles)
      rospy.loginfo('setting robot angle> ' + str(self.angles[nang]))
      userdata.selected = self.angles[nang]
      
      return 'succeeded'


class MoveFace(smach.State):

    def __init__(self,angle_face):
        smach.State.__init__(self, outcomes=['rotate','succeeded','aborted','preempted'])
        self.c = 0
        self.rotation = 0
        self.angle= angle_face

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveFace')

        if self.c != 0:
          if self.c%3 == 0:
            FaceOrder.ChangeFace(0)
            self.c+=1
            return 'rotate' 
          else:
            if self.c%3 == 1:
              FaceOrder.ChangeFace(self.angle*-1)
            else:
              FaceOrder.ChangeFace(self.angle)


        self.c+=1
        return 'succeeded'

class PubShelf(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.shelf = benpy.ServiceProxy('/bender/pcl/shelf_detector/enable', Onoff)
        self.pub_collision = benpy.ServiceProxy('/bender/vision_interface/CollisionObject/ShelfColission', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state PubShelf')

        FaceOrder.ChangeFace(0)
        rospy.sleep(0.5)

        self.shelf(True)
        rospy.sleep(0.5)

        self.pub_collision()
        self.shelf(False)
        
        return 'succeeded'

    
def getInstance():

      sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
            input_keys=['time_init'])

      sm.userdata.map_name = 'map.sem_map'
      sm.userdata.sm_shelf_pose = PoseStamped()

      # rgbd
      rgbd_angles = [55, 40, 25]
      sm.userdata.current_rgbd_angle = 30
      sm.userdata.ang_final = 40
      sm.userdata.ang_prepare = 40

      #rotate face
      angle_face = 55
      # rotacion
      robot_angles = [165, 152]
      sm.userdata.current_robot_angle = 0
      sm.userdata.turn_around_angle = 100

      sm.userdata.move_dist = 0.5
      sm.userdata.approach_distance = 0.55
      
      with sm:
            
            smach.StateMachine.add('SETUP', Setup(),
                transitions={'succeeded':'SELECT_RGBD_ANGLE'}
            )
            
            ## scan looking for a shelf
            smach.StateMachine.add('SELECT_RGBD_ANGLE', SelectRGBDAngle(rgbd_angles),
                transitions={'succeeded':'MOVE_RGBD',
                             'yaw':'ROTATE_HEAD'},
                remapping={'selected':'current_rgbd_angle'}
            )

            smach.StateMachine.add('MOVE_RGBD', MoveAsus.getInstance(),
                transitions={'succeeded':'CHECK_SHELF_EXISTENCE'},
                remapping={'deg_angle':'current_rgbd_angle'}
            )   

            smach.StateMachine.add('ROTATE_HEAD',MoveFace(angle_face),
                    transitions ={'succeeded':'MOVE_RGBD',
                                  'aborted':'MOVE_RGBD',
                                  'rotate':'SELECT_ROBOT_ANGLE'},
            )

            smach.StateMachine.add('CHECK_SHELF_EXISTENCE', SearchShelf(),
                transitions={'succeeded':'GO_TO_SHELF',
                             'fail':'SELECT_RGBD_ANGLE'}
            )
            ## failed: Rotate Robot and try again
            smach.StateMachine.add('SELECT_ROBOT_ANGLE', SelectRobotAngle(robot_angles),
                transitions={'succeeded':'TURN_AROUND'},
                remapping={'selected':'current_robot_angle'}
            )
            smach.StateMachine.add('TURN_AROUND',RotateRobot.getInstance(),
                transitions = {'succeeded':'SELECT_RGBD_ANGLE',
                               'aborted':'SELECT_RGBD_ANGLE'},
                remapping   = {'angle':'turn_around_angle'}
            )
            
            ## go to pre shelf pose
            smach.StateMachine.add('GO_TO_SHELF', GoToPoseStamped.getInstance(),
                transitions = {'succeeded':'MOVE_RGBD_PREPARE',
                               'aborted':'GO_TO_SHELF'},
                remapping = {'goal_pose':'sm_shelf_pose'} 
            )    
            ## prepare approach
            smach.StateMachine.add('MOVE_RGBD_PREPARE', MoveAsus.getInstance(),
                transitions={'succeeded':'PUB_SHELF'},
                remapping={'deg_angle':'ang_prepare'}
            )
            
            smach.StateMachine.add  ('PUB_SHELF' , PubShelf(),
                transitions  =  {'succeeded':'MOVE_X'}
            )
            ## approach behavior
            smach.StateMachine.add('MOVE_X', MoveBaseX.getState(),
                transitions={'succeeded':'GET_CLOSER'},
                remapping={'d':'move_dist'}
            )
            smach.StateMachine.add('GET_CLOSER', ApproachToPlane.getInstance(),
                transitions={'succeeded':'MOVE_RGBD_FINAL'},
                remapping={'distance':'approach_distance'})
            
            ## prepare approach
            smach.StateMachine.add('MOVE_RGBD_FINAL', MoveAsus.getInstance(),
                transitions={'succeeded':'succeeded'},
                remapping={'deg_angle':'ang_final'}
            )

      #ud = smach.UserData()
      #sm.set_initial_state(['SELECT_ROBOT_ANGLE'], ud)

      return sm


# main
if __name__ == '__main__':

      rospy.init_node('approach_shelf')

      sm = getInstance()

      # introspection server
      sis = smach_ros.IntrospectionServer('approach_shelf', sm, '/APPROACHSHELFSM')
      sis.start()
      outcome = sm.execute()
      sis.stop()