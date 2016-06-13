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
from bender_macros.nav import ApproachToTable
from bender_macros.vision import PositionAndDetect
from bender_macros.vision import Detect
#from bender_behaviors.Stage1.GPSR import Grasp
from bender_macros.speech import Talk

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        Talk.getInstance('I will find the object', 3)


        rospy.sleep(0.5)
        return 'succeeded'



# define state ChooseObject
class FindObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found','succeeded','aborted','preempted','canceled'],
             io_keys=['object_name','object_pose','object_id','object_image','object_position','report'])
        self.c = 0
        self.c_max = 3
        self.bridge = CvBridge()

    def execute(self, userdata):
        rospy.loginfo('Executing state FindObject')


        if userdata.object_name in userdata.object_id:
                id_obj = userdata.object_id.index(userdata.object_name)
                userdata.object_pose = userdata.object_position[id_obj]

                try:
                    cv_image = self.bridge.imgmsg_to_cv2(userdata.object_image[id_obj], "bgr8")
                except CvBridgeError, e:
                    print e


                # cv2.imshow("deteccion", cv_image)
                # cv2.waitKey(3)
                rospy.loginfo(userdata.object_name + "found ")
                sp = "I found the "+userdata.object_name
                Talk.getInstance(sp,len(sp)/9)
                userdata.report = sp

                return 'succeeded'

        if self.c > self.c_max:
            sp = "I didnt find the "+userdata.object_name
            Talk.getInstance(sp,len(sp)/9)
            userdata.report = sp
            return 'canceled'
        self.c += 1  

        return 'not_found' 

    



def getInstance():

    sm = smach.StateMachine(
        outcomes=['not_found','succeeded','aborted','preempted'],
        input_keys=['object_name'],
        output_keys=['report','object_pose'])
    
    
    sm.userdata.object_counter = 0
    sm.userdata.object_position = []
    sm.userdata.object_id = []
    sm.userdata.object_image = []
    sm.userdata.object_type = []
    sm.userdata.object_status = []
    sm.userdata.object_class_location = ""

    sm.userdata.object_pose = PoseStamped()
    sm.userdata.selected_type = ""
    sm.userdata.report = "I take the object"
    sm.userdata.use_waist = False
    sm.userdata.use_off = False
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
               transitions={'succeeded':'POSITION_AND_DETECT'})

        smach.StateMachine.add('POSITION_AND_DETECT',Detect.getInstance(),#PositionAndDetect.getInstance(),
               transitions={'succeeded':'FIND_OBJECT',
                            'notDetected':'FIND_OBJECT'},
               remapping={'use_waist':'use_waist',
                          'object_position':'object_position',
                          'object_id':'object_id',
                          'object_image':'object_image'})

        smach.StateMachine.add('FIND_OBJECT',FindObject(),
               transitions={'succeeded':'succeeded',
                            'not_found' : 'POSITION_AND_DETECT',
                            'canceled':'not_found'})

   

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('FindObject')

    sm = getInstance()
    ud = smach.UserData()
    ud.object_name = "musculo"
    # introspection server
    sis = smach_ros.IntrospectionServer('FindObject', sm, '/FindObject_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
