#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_utils.ros import benpy
from bender_srvs.srv import ID
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import FaceInfoResponse
from bender_srvs.srv import FaceRecognition
from bender_srvs.srv import FaceRecognitionRequest
from bender_srvs.srv import FaceRecognitionResponse
from sensor_msgs.msg import Image

#  - - - - macros - - - -
from bender_macros.speech import Talk
import sensor_msgs


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded','aborted','preempted'],
            output_keys=['timeinit'])
        
    def execute(self, userdata):
        Talk.getInstance("Starting enroll face",2.5)

        return 'succeeded'


class GetFreeId(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'],
                             output_keys=['free_face_id'])

        #self.face_id_client = rospy.ServiceProxy('/bender_vision/face_recognizer/add',FaceRecognition)
        self.face_id = 10

    def execute(self, userdata):
        rospy.loginfo('Executing state: GET_FREE_ID')

        #self.face_id_client.wait_for_service()

        userdata.free_face_id = self.face_id
        return 'succeeded'

class DetectFaces(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','srv_except'],
                             output_keys=['main_face'])

        self.face_detection_client = benpy.ServiceProxy('/bender/vision/face_detector/detect_face',FaceInfo)
        self.min_area = 150*150

    def get_main_face(self, bboxes):
        
        max_area = 0
        max_idx  = -1
        for k in xrange(len(bboxes)):
            
            box = bboxes[k]
            area = box.width*box.height
            
            rospy.logwarn("bbox w,h = " + str(box.width) + ", " + str(box.height))
            
            if area > max_area:
                max_area = area
                max_idx = k
                
        if max_area < self.min_area:
            return -1
        return max_idx

    def execute(self, userdata):
        rospy.loginfo('Executing state: DETECT_FACES')

        self.face_detection_client.wait_for_service()
  
        try:
            response = self.face_detection_client(return_images = True)

            if response.n_faces > 0:
                
                main_idx = self.get_main_face(response.BBoxes)
                if main_idx < 0:
                    return "aborted"
                
                userdata.main_face = response.faces[main_idx]
                rospy.loginfo('number of detected faces: ' + str(response.n_faces))
                return 'succeeded'

            else:
                rospy.loginfo('number of detected faces: 0')
                return 'aborted'

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'srv_except'


class HandleZeroFaces(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

        self.max_retry_times = 10
        self.current_retry_times = 0

    def execute(self, userdata):
        
        rospy.loginfo('Executing state: HANDLE_ZERO_FACES')

        self.current_retry_times = self.current_retry_times + 1
        if self.current_retry_times > self.max_retry_times:
            return 'aborted'

        Talk.getInstance("Please, look at my eyes", 3)
        return 'succeeded'
    

class AddFace(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'],
                             input_keys=['face_id','face_im'])

        self.face_add_client = benpy.ServiceProxy('/bender/vision/face_recognizer/add',FaceRecognition)

    def execute(self, userdata):
        rospy.loginfo('Executing state: ADD_FACE')

        self.face_add_client.wait_for_service()
        
        req = FaceRecognitionRequest()
        req.add_face_index = userdata.face_id
        req.use_image = True
        req.image = userdata.face_im
        
        try:
            response = self.face_add_client(req)            
            return 'succeeded'

        except rospy.ServiceException, e:
            print "failed to save image: %s"%e
            return 'aborted'


class HandleAddFace(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done','need_more_imgs'],
                             input_keys=['n_enroll_images'])

        self.correct_images = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state: HANDLE_ADD_FACE')

        self.correct_images = self.correct_images + 1
        rospy.loginfo("enroll image number (" + str(self.correct_images) + str(") ready"))
        rospy.sleep(0.3)

        if self.correct_images < userdata.n_enroll_images:
            return 'need_more_imgs'
        else:
            return 'done'


def getInstance():


    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['n_enroll_images'],
                            output_keys=['enroll_face_id'])

    sm.userdata.enroll_face_id = -1
    sm.userdata.sm_main_face = Image()

    with sm:
        smach.StateMachine.add('SETUP',setup(),
                transitions = {'succeeded':'GET_FREE_ID'},
            )
        smach.StateMachine.add('GET_FREE_ID',GetFreeId(),
                           transitions={'succeeded':'DETECT_FACES',
                                        'aborted':'aborted'},
                           remapping={'free_face_id':'enroll_face_id'})

        smach.StateMachine.add('DETECT_FACES',DetectFaces(),
                           transitions={'succeeded':'ADD_FACE',
                                        'aborted':'HANDLE_ZERO_FACES',
                                        'srv_except':'aborted'},
                           remapping={'main_face':'sm_main_face'})

        smach.StateMachine.add('HANDLE_ZERO_FACES',HandleZeroFaces(),
                           transitions={'succeeded':'DETECT_FACES',
                                        'aborted':'aborted'})

        smach.StateMachine.add('ADD_FACE',AddFace(),
                           transitions={'succeeded':'HANDLE_ADD_FACE',
                                        'aborted':'DETECT_FACES'},
                           remapping={'face_id':'enroll_face_id',
                                      'face_im':'sm_main_face'})

        smach.StateMachine.add('HANDLE_ADD_FACE',HandleAddFace(),
                           transitions={'done':'succeeded',
                                        'need_more_imgs':'ADD_FACE'},
                           remapping={'n_enroll_images':'n_enroll_images'})

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('enroll_face')

    sm = getInstance()
    ud = smach.UserData()
    ud.n_enroll_images = 10

    # introspection server
    sis = smach_ros.IntrospectionServer('enroll_face', sm, '/ENROLL_FACE_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
