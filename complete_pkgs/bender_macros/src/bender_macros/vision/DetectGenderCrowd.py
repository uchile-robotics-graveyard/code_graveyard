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

class AnalyzeCrowdGender(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                            input_keys  = ['crowd_size_hint'],
                            output_keys = ['n_women','n_men'])
        
        self.face_detection_client     = benpy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
        self.gender_recognition_client = benpy.ServiceProxy('/gender_recognizer/recognize', GenderDetection)
        
        # DEFINES
        self.GENDER_UNKOWN = "unknown"
        self.GENDER_MEN    = "man"
        self.GENDER_WOMEN  = "woman"
        
        self.recog_attempts = 3
        self.detec_attempts = 3

        self.min_size_face = 50*50
        self.distance_between_face = 5*5

    def execute(self, userdata):
        
        #Talk.getInstance("can you look at me, please?... i like humans", 5)
        
        # prepare detection srv
        det_srv = FaceInfoRequest()
        det_srv.use_image = False
        det_srv.return_full_image = True
        det_srv.return_images = True
        
        # gender recognition
        best_n_men   = 0
        best_n_women = 0
        rec_attempt  = 0
        while rec_attempt < self.recog_attempts:
            
            # detect faces and keep the biggest set
            det_attempt = 0
            best_detection = FaceInfoResponse()
            while det_attempt < self.detec_attempts:
                
                # detect
                detected = self.face_detection_client(det_srv)
                detected = self.bestnface(detected, userdata.crowd_size_hint)

                # keep the best
                if best_detection.n_faces < detected.n_faces:
                    best_detection = detected
                    
                det_attempt+=1
                rospy.sleep(0.5) # to get different images
                
            rospy.loginfo("attempt (" + str(rec_attempt)+ ", found (" + str(best_detection.n_faces) + ") faces")
            
            # -- gender recognition --
            n_women = 0
            n_men   = 0
            
            # fill srv
            gender_srv = GenderDetectionRequest()
            gender_srv.image = best_detection.image
            gender_srv.faces = best_detection.faces
            gender_srv.BBoxes = best_detection.BBoxes
            gender_srv.eyes_positionL = best_detection.eyes_positionL
            gender_srv.eyes_positionR = best_detection.eyes_positionR
            gender_srv.n_faces = best_detection.n_faces
            gender_srv.simulate_eyes = True
            
            # recognize
            person_gender = None
            try:
                person_gender = self.gender_recognition_client(gender_srv)

            except rospy.ServiceException as e:
                rospy.logwarn("GenderRecognition: Ups. Failed to call the gender recognize server. why?: " + str(e))
                rec_attempt+=1
                continue
            
            # assign the difference to the men
            n_total = person_gender.n_men + person_gender.n_women
            diff = userdata.crowd_size_hint - n_total
            n_men = person_gender.n_men
            if diff > 0:
                 n_men += diff
            n_women = person_gender.n_women
            
            if n_total > (best_n_men + best_n_women):
                best_n_men = n_men
                best_n_women = n_women
            
            rospy.loginfo("recognized -> men: " + str(person_gender.n_men) +
                           "women: " + str(person_gender.n_women) + ", unknown: " + str(diff))
            rec_attempt+=1
        
        rospy.loginfo("recognition selected -> men: " + str(best_n_men) + "women: " + str(best_n_women))
        
        userdata.n_men   = best_n_men
        userdata.n_women = best_n_women
        return 'succeeded'


    def bestnface(self, detection, n):

        faces = detection.faces
        boxes = detection.BBoxes
        eyes_positionL = detection.eyes_positionL
        eyes_positionR = detection.eyes_positionR

        if len(faces)==0:
            return detection
        print "caras con sueno"+str(len(faces))

        faces_out = []
        boxes_out = []
        eyes_positionL_out = []
        eyes_positionR_out = []
        facessave =[]

        for i in xrange(n):
            maxface = faces[0]
            k =0
            ksave=0
            for face in faces:
                if face.height*face.width > maxface.height*maxface.width and k not in facessave:
                    maxface = face
                    ksave=k
                k +=1
            if maxface.height*maxface.width > self.min_size_face:
                save = False
                if len(facessave)>0:
                    if (maxface.height*maxface.width - faces_out[-1].height*faces_out[-1].width) < self.distance_between_face:
                        save = True
                if len(facessave)==0:
                    save = True
                if save:
                    facessave.append(ksave)
                    faces_out.append(maxface)
                    boxes_out.append(boxes[ksave])
                    eyes_positionL_out.append(eyes_positionL[ksave])
                    eyes_positionR_out.append(eyes_positionR[ksave])


        detection.faces = faces_out
        detection.BBoxes = boxes_out
        detection.eyes_positionL = eyes_positionL
        detection.eyes_positionR = eyes_positionR
        detection.n_faces = len(faces_out)

        return detection
#     def sortlists(self, gender, poses):
#          
#         posesy = []
#         for i in poses:
#             posesy = i.pose.position.y
#  
#         genderout=[]
#         posesout=[]
#         temp=[]
#         while len(posesy)>0:
#             v = posesy.index(min(posesy))
#             if len(gender)>v:
#                 genderout.append(gender.pop(v))
#             else:
#                 if len(gender)>0:
#                     genderout.append(gender.pop(0))
#                 else:
#                     genderout.append("")
#             posesout.append(poses.pop(v))
#             temp.append(posesy.pop(v))
#  
#         return genderout, posesout


def getInstance():
    
    sm = AnalyzeCrowdGender()
    return sm

# main
if __name__ == '__main__':

    try:
        
        rospy.init_node('detect_gender_crowd')
        
        
        sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                input_keys=['crowd_size_hint'],
                                output_keys=['n_women','n_men'])
        
        with sm:
            
            smach.StateMachine.add('DETECT_GENDER_CROWD', AnalyzeCrowdGender(),
                transitions={'succeeded':'succeeded'}
            )
        
        ud = initUserData()
        a = PoseStamped()
        b = PoseStamped()
        p = []
        p.append(a)
        p.append(b)
        ud.crowd_size_hint = p
    
        # introspection server
        sis = smach_ros.IntrospectionServer('detect_gender_crowd', sm, '/DETECT_GENDER_CROWD_SM')
        sis.start()
        outcome = sm.execute(ud)
        sis.stop()
        
    except rospy.ROSInterruptException:
        pass
    