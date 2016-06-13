#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
import cv2

from math import sqrt
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_macros.speech import Talk 
from bender_utils.ros import benpy
from bender_macros.head import FaceOrder
from cv_bridge import CvBridge, CvBridgeError

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #



class RecognizeGender(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             output_keys=['person_gender'])
        self.face_detection = benpy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
        self.gender_reco = benpy.ServiceProxy('/gender_recognizer/recognize', GenderDetection)

        self.contW = 0
        self.contM = 0
        self.contmax = 2
        
        
    def execute(self, userdata):

        rospy.loginfo('Executing state class Recognize Gender')

        Talk.getInstance("Please, look to my eyes to check your gender ", 3)
        
        reqF = FaceInfoRequest()
        reqF.use_image = False
        reqF.return_images = True
        reqF.return_full_image = True
        cont = 0

        self.contW = 0
        self.contM = 0
        
        # default gender
        userdata.person_gender = 'male'

        while cont < self.contmax*2:
            try:
                resp = self.face_detection(reqF)
                rospy.loginfo("found " + str(resp.n_faces) + " faces")

            except rospy.ServiceException, e:
                return 'aborted'

            id_face = self.get_main_face(resp.faces)

            if id_face != -1:
                #try:
                #    bridge = CvBridge()
                #    cv_rgb_image = bridge.imgmsg_to_cv2(resp.faces[id_face], "bgr8")
                #    # cv2.imshow("faceas", cv_rgb_image)
                #    # cv2.waitKey(10)
                #except Exception, e:
                #    rospy.logerr("failed to convert ROS image to OpenCV image. why?: " + str(e))

                req = GenderDetectionRequest()
                req.image = resp.image
                req.faces = [resp.faces[id_face]]
                req.BBoxes = [resp.BBoxes[id_face]]
                req.eyes_positionL = [resp.eyes_positionL[id_face]]
                req.eyes_positionR = [resp.eyes_positionR[id_face]]
                req.n_faces = 1
                req.simulate_eyes = False

                try:
                    resp=self.gender_reco(req)
                    print "hombre : "+str(resp.n_men)
                    print "mujer : "+str(resp.n_women)

                    if resp.n_men + resp.n_women == 1:
                        self.contW  = self.contW + resp.n_women
                        self.contM  = self.contM + resp.n_men
                        cont = cont + 1
                    elif len(resp.gender)>1:
                        cont = cont + 1
                        if resp.gender[0] == "female":
                            self.contW  = self.contW + 1
                        if resp.gender[0] == "male":
                            self.contM  = self.contM + 1

                except rospy.ServiceException, e:
                    return 'aborted'
                

        if self.contW + self.contM > 0 and ( self.contM > self.contmax or self.contW > self.contmax):
            gender_str = 'male'
            sp = " we can be friends"
            face = "happy2"
            if self.contW > self.contM:
                gender_str = 'female'
                sp = "and very pretty indeed"
                face = "1313"
            userdata.person_gender = gender_str
            Talk.getInstance("Hi operator, I see you are a " + gender_str + " human", 3)
            Talk.getInstance(sp, len(sp)/8)
            FaceOrder.ChangeFace(face)

            return 'succeeded'

        return 'aborted'

    def get_main_face(self, faces):
        if len(faces) == 0:
            return -1

        idF = 0
        areaF = 0
        i = 0
        for im in faces:
            if im.height*im.width > areaF:
                areaF = im.height*im.width
                idF = i
            i+=1
        return idF


def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            output_keys=['person_gender'])
    
    with sm:

        smach.StateMachine.add('RECO_GENDER', RecognizeGender(),
            transitions = {'succeeded':'succeeded'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('detect_gender')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('detect_gender', sm, '/DETECT_GENDER_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
