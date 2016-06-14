#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

import cv2
from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from bender_utils.ros import benpy

#  - - - - macros - - - -
from bender_macros.speech import Talk

class DetectState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                            input_keys=['crowd_images','crowd_ROIs_boxes','crowd_ROIs'],
                            io_keys=['crowd_states'])
        
        self.state_detection_client = benpy.ServiceProxy('/bender/perception/state/StateDetectorBoxes', ImageDetection)
        

    def execute(self, userdata): 

        rospy.loginfo('Executing state: state_RECOGNITION')
        Talk.getInstance("I am identifying your poses, please keep still",2)

        userdata.crowd_states = []

        c = 0
        for i in userdata.crowd_images:
            req = ImageDetectionRequest()
            req.detection_boxes = []
            req.detections = []

            req.detection_boxes.append(userdata.crowd_ROIs_boxes[c])
            req.detections.append(userdata.crowd_ROIs[c])

            print req.detection_boxes
            print req.detections
            #req.image = i

            try:
                res = self.state_detection_client(req)
            except:
                return 'aborted'
                
            print res.state
            if res.state == "stand":
                userdata.crowd_states.append("standing")
            else:
                userdata.crowd_states.append("sitting")
            c+=1

        print userdata.crowd_states
        return 'succeeded'



def getInstance():

    return DetectState()


# main
if __name__ == '__main__':

    rospy.init_node('detect_state_crowd')

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                input_keys=['crowd_images','crowd_ROIs_boxes','crowd_ROIs'],
                output_keys=['crowd_states'])
    
    with sm:

        smach.StateMachine.add('DETECT_GENDER_CROWD', DetectState(),
            transitions={'succeeded':'succeeded',
                         'aborted':'succeeded'})

    # introspection server
    sis = smach_ros.IntrospectionServer('detect_state_crowd', sm, '/DETECT_STATE_CROWD_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()

