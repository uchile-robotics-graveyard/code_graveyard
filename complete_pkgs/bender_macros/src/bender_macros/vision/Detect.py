#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from math import pi
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_utils.ros import benpy
from bender_arm_control.srv import HeadPosition, HeadPositionRequest, HeadPositionResponse

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

# define state Setup
class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                            input_keys =['use_waist'])
        self.path = "/bender/pcl/SiftDetectorseg"

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')

        srv= self.path+"Head/"
        if userdata.use_waist:
            srv= self.path+"Waist/"
        print srv

        try:
            siftactive = benpy.ServiceProxy(srv+'Active', Onoff)
            resp2 = siftactive(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'preempted'

        return 'succeeded'

# define state Detect
class Detect(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','Finish','notDetected','aborted','preempted'],
                            input_keys =['use_waist','use_off'],
                             output_keys=['object_position','object_id','object_image','object_status'])
        self.c = -1
        self.c_max = 3
        self.path = "/bender/pcl/SiftDetectorseg"

    def execute(self, userdata):
        rospy.loginfo('Executing state Detect')
        rospy.sleep(0.5)
        
        self.c = self.c +1
        srv= self.path+"Head/"

        if userdata.use_waist:
            srv= self.path+"Waist/"

        try:
            detect = benpy.ServiceProxy(srv+'Recognize', ObjectDetection)
            detected_obj = detect()

            #print detected_obj.name
            #print detected_obj.pose.position
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'preempted'

        if len(detected_obj.name)==0 and self.c<self.c_max:
            return 'notDetected'


        # try:
        #     resp1 = siftactive(False)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e
        #     return 'preempted'
        if userdata.use_off:
            try:
                siftactive = benpy.ServiceProxy(srv+'Active', Onoff)
                resp1 = siftactive(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return 'preempted'

	
        rospy.loginfo('Objects : ' + str(len(detected_obj.name)) )

        userdata.object_position = detected_obj.pose
        userdata.object_id = detected_obj.name
        userdata.object_image = detected_obj.img

        if  len(detected_obj.name)>0:
            return 'succeeded'

        return 'Finish'
        


#use_waist(boolean) = true : usar waist, false: usar head
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','notDetected','aborted','preempted'],
                            input_keys=['use_waist','use_off'],
                            output_keys=['object_position','object_id','object_image'])
    sm.userdata.object_position = []
    sm.userdata.object_id = []
    sm.userdata.object_image = []
    sm.userdata.object_status = []


    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                           transitions={'succeeded':'DETECT'})

        smach.StateMachine.add('DETECT',Detect(),
                           transitions={'succeeded':'succeeded',
                                        'notDetected':'DETECT',
                                       'Finish':'notDetected',
                                       'preempted':'DETECT'},
                           remapping={'object_position':'object_position',
                                      'object_id':'object_id'})

    return sm
 
 
# main
if __name__ == '__main__':

    rospy.init_node('Detect')

    sm = getInstance()
    ud = smach.UserData()
    ud.use_waist = False;
    ud.use_off = False;
    # introspection server
    sis = smach_ros.IntrospectionServer('Detect', sm, '/DETECT_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
