#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import smach
import smach_ros
import cv2
from bender_srvs.srv import *
from bender_msgs.msg import *
from geometry_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_macros.nav import GoToPlace
from bender_macros.vision import PositionAndDetect
from bender_macros.vision import Detect
from bender_behaviors.Stage1.GPSR import Grasp
from bender_macros.speech import Talk
from bender_behaviors.Stage1.ManipulationTest import DetectObjects
from bender_macros.skills.Approach import ApproachShelf
from bender_behaviors.Stage1.ManipulationTest import ManipulateObjects
from bender_arm_control.arm_commander import Limb

from bender_macros.nav import ApproachToTable


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from bender_macros.arm import GraspCartesian_shelf

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
            output_keys = ['time_init']
        )
        self.octomap = GraspCartesian_shelf.OctomapManager()

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        self.octomap.stop()

        userdata.time_init =  time.time()

        return 'succeeded'


def getInstance():

    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'])
    
    

    sm.userdata.detected_objects = []
    sm.userdata.selected_object_list = []
    sm.userdata.contador = 0
    sm.userdata.counter = 0

    # save report
    sm.userdata.time_init = 0

    sm.userdata.selected_objects = []
    sm.userdata.object_id = []
    sm.userdata.object_position = []
    sm.userdata.object_image = []

    sm.userdata.selected_objects_name = []
    sm.userdata.selected_objects_pose = []
    sm.userdata.selected_manipulate = []#True object manipulado o False object no manipulado


    sm.userdata.map_name = 'map.sem_map'
    sm.userdata.place_name = 'shelf'
    sm.userdata.distance_approach_table = 0.55

    with sm:

        smach.StateMachine.add  ('SETUP', Setup(),
          transitions  =  {'succeeded':'APPROACH_SHELF'}
        )
        smach.StateMachine.add  ('APPROACH_SHELF', ApproachToTable.getInstance(),
               transitions={'succeeded':'DETECT_OBJECTS'},
               remapping={'table_name':'place_name',
                          'map_name':'map_name',
                          'desired_distance':'distance_approach_table'}
        )

        smach.StateMachine.add  ('DETECT_OBJECTS', DetectObjects.getInstance(),#'selected_objects','objects_name','objects_pose','objects_img
            transitions  =  {'succeeded':'MANIPULATE',
                     'no_detections':'DETECT_OBJECTS',
                     'aborted':'DETECT_OBJECTS'},
            remapping   =   {'object_id':'object_id',
                            'object_position':'object_position',
                            'object_image':'object_image',
                            'selected_objects_name':'selected_objects_name',
                            'selected_objects_pose':'selected_objects_pose',}
        )
        
        smach.StateMachine.add  ('MANIPULATE', ManipulateObjects.getInstance(),
            transitions  =  {'succeeded':'succeeded',
                            'preempted' : 'MANIPULATE'},
            remapping   =   {'selected_objects_name':'selected_objects_name',
                            'selected_objects_pose':'selected_objects_pose',
                            'selected_manipulate':'selected_manipulate'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('manipulation')

    sm = getInstance()

    sis = smach_ros.IntrospectionServer('manipulation', sm, '/MANIPULATION_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
