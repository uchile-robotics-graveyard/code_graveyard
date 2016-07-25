#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from std_srvs.srv import Empty
from bender_srvs.srv import *
from bender_msgs.msg import *

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


# define state DetectObjects
class DetectObjects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','no_detections','aborted','preempted'],
             output_keys=['object_positions','object_ids','object_types','object_status'])
        
        # service clients
        self.sift_on_client = rospy.ServiceProxy('/siftOn', siftOn)
        self.sift_off_client = rospy.ServiceProxy('/siftOff', siftOff)
        self.plane_on_client = rospy.ServiceProxy('/planeOn', planeOn)
        self.plane_off_client = rospy.ServiceProxy('/planeOff', planeOff)
        self.reset_obj_client = rospy.ServiceProxy('/arm_vision_interface/reset_obj', Empty)
        self.detect_obj_client = rospy.ServiceProxy('/arm_vision_interface/detect_obj', ObjectDetection)

        # seconds to wait for detections
        self.N = 5
        
        # offsets for detection transformation (in [cm])
        self.offset_x = 3
        self.offset_y = - 23.5
        self.offset_z = 123.5
        
        # distance limits for manipulation
        self.max_y = 84 

    def execute(self, userdata):
        rospy.loginfo('Executing state Detect')
                
        try:    
            # wait services
            rospy.loginfo("Waiting for '/siftOn' service to come up")
            self.sift_on_client.wait_for_service()
    
            rospy.loginfo("Waiting for '/planeOn' service to come up")
            self.plane_on_client.wait_for_service()
            
            rospy.loginfo("Waiting for '/arm_vision_interface/reset_obj' service to come up")
            self.reset_obj_client.wait_for_service()
            
            rospy.loginfo("Waiting for '/arm_vision_interface/detect_obj' service to come up")
            self.detect_obj_client.wait_for_service()
            
            rospy.loginfo("Waiting for '/siftOff' service to come up")
            self.sift_off_client.wait_for_service()
    
            rospy.loginfo("Waiting for '/planeOff' service to come up")
            self.plane_off_client.wait_for_service()
    
            # turn sift-detector on            
            self.sift_on_client(0)
            
            # turn plane-detector on
            self.plane_on_client(0)
            
            # wait for detectors to come up ~1[s]
            time.sleep(1)

            # clear current objects
            self.reset_obj_client()            
            
            # wait N[s] to get a lot of detections
            time.sleep(self.N)
           
            # get current detections
            detected_objects = self.detect_obj_client()
            
            # turn plane-detector off
            self.plane_off_client(0)
            
            # turn sift-detector off            
            self.sift_off_client(0)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        # process result
        # - - - - - - - - - - - - - - - - - - - - - - - -
        
        # no detections
        if len(detected_objects.name)==0:
            return 'no_detections'
        
        print detected_objects.name
        print detected_objects.type
        
        # save avaliable detections
        userdata.object_positions = {'x':detected_objects.x, 'y':detected_objects.y, 'z':detected_objects.z}
        userdata.object_ids = detected_objects.name
        userdata.object_types = detected_objects.type

        # analize range
        range_status = []
        for i in range(len(detected_objects.name)):
            
            distance_right = sqrt(
                  (detected_objects.x[i] - self.offset_x)**2 + 
                  (detected_objects.y[i] - self.offset_y)**2 +
                  (detected_objects.z[i] - self.offset_z)**2
            )
            
            distance_left = sqrt(
                 (detected_objects.x[i] - self.offset_x)**2 +
                 (detected_objects.y[i] + self.offset_y)**2 +
                 (detected_objects.z[i] - self.offset_z)**2
            )
            
            if distance_left > self.max_y and distance_right > self.max_y:
                range_status.append("out_of_range")
            else:
                range_status.append("in_range")

        userdata.object_status = range_status

        return 'succeeded'

def getInstance():

    sm = smach.StateMachine(
            outcomes=['succeeded','no_detections','aborted','preempted'],
            output_keys=['object_positions','object_ids','object_types','object_status'])
    
    sm.userdata.object_positions = {}
    sm.userdata.object_ids = []
    sm.userdata.object_types = []
    sm.userdata.object_status = []

    with sm:

        smach.StateMachine.add('DETECT',DetectObjects(),
               transitions={'succeeded':'succeeded',
                            'no_detections':'no_detections'},
               remapping={'object_positions':'object_positions',
                          'object_ids':'object_ids',
                          'object_types':'object_types',
                          'object_status':'object_status'})

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('detect_objects')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('detect_objects', sm, '/DETECT_OBJECTS_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
