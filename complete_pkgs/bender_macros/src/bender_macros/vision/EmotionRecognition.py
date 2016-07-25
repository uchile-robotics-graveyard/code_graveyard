#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import math
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

# #  - - - - macros - - - -

from bender_macros.speech import Talk

from bender_macros.head import FaceOrder


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #



class EmotionRecognition(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                             output_keys=['person_emotion'])

        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)

    def execute(self, userdata):

        rospy.loginfo('Executing state: EMOTION_RECOGNITION')
        #sp = "my friend, I will identify how you feel"
        #Talk.getInstance(sp,len(sp)/8)

        emotion_recognition = benpy.ServiceProxy('/bender/vision/emo_detector/detect_emotion', DetectState)

        max_score = 0
        max_emotion = ""
        for i in xrange(5):

            try:
                resp=emotion_recognition()
    
                if resp.score > max_score:
                    max_score = resp.score
                    max_emotion = resp.state
    
                if resp.score > 11:
                    userdata.person_emotion = resp.state
                    self.BenderFace(resp.state)
                    return 'succeeded'
            
            except rospy.ServiceException, e:
                return 'preempted'
            
        
        userdata.person_emotion = max_emotion
        return 'succeeded'

    def BenderFace(self,emotion):
                
        em = "happy1"
        if emotion == "ANGER":
            em = "angry2"
        elif emotion == "CONTEMPT":
            em = "1313"
        elif emotion == "DISGUST":
            em = "angry1"
        elif emotion == "FEAR":
            em = "sad1"
        elif emotion == "HAPPY":
            em = "happy2"
        elif emotion == "SADNESS":
            em = "sad2"
        elif emotion == "SURPRISE":
            em = "surprise"

        FaceOrder.ChangeFace(em)



def getInstance():
    
    return EmotionRecognition()

# main
if __name__ == '__main__':

    rospy.init_node('emotion_recognition')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('emotion_recognition', sm, '/EMOTION_RECOGNITION_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
