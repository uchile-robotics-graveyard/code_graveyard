#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import cv2

from bender_utils.ros import benpy
from bender_srvs.srv import ID
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import FaceInfoRequest
from bender_srvs.srv import FaceInfoResponse
from bender_srvs.srv import FaceRecognition
from bender_srvs.srv import FaceRecognitionRequest
from bender_srvs.srv import FaceRecognitionResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#  - - - - macros - - - -
from bender_macros.speech import Talk
from bender_macros.speech import TalkState
from bender_macros.head import FaceOrder
from bender_macros.head import MoveAsus
import sensor_msgs


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class SubsImage(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                              outcomes=['succeeded','aborted','preempted'],
                              output_keys=['image'])
        
        self.rgb_topic   = '/bender/sensors/rgbd_head/rgb/image_raw'
        self.rgb_subs = None
       
        # data 
        self.rgb_image = None
        self.bridge = CvBridge()
        self.cv_rgb_image = []
        
        # control
        self.got_rgb_img = False
        
        # start on "pause" state
        self.pause_dataflow()

    def init_dataflow(self):
        
        rospy.loginfo("initializing rgbd data flow")
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if not self.rgb_subs:
            rospy.loginfo("...")
            self.rgb_subs   = rospy.Subscriber(self.rgb_topic, sensor_msgs.msg.Image, self.rgb_cb)
            rospy.sleep(0.1)
        rospy.loginfo("rgd flow initialized")

    def pause_dataflow(self):
        rospy.loginfo("pausing rgbd data flow")
        self.got_rgb_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if self.rgb_subs:
            self.rgb_subs.unregister()
        self.rgb_subs   = None
        rospy.loginfo("rgd flow paused")

    def rgb_cb(self,msg):
        rospy.loginfo("got new rgbd msg")
        self.rgb_image = msg
        self.got_rgb_img = True

    def get_rgbd_data(self):
        
        self.init_dataflow()
        if not self.got_images:
            
            rospy.sleep(0.5)
            
            while not self.got_rgb_img:
                rospy.sleep(0.1)
                
            self.got_images = True

        if not self.got_images_to_cv2:
            try:
                self.cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
            except rospy.ServiceException as e:
                rospy.logerr("EnrollRGBD: failed to convert ROS image to OpenCV image. why?: " + str(e))
                self.pause_dataflow()
                return False
            self.got_images_to_cv2 = True
        
        self.pause_dataflow()
        return True

    def execute(self, userdata):

        if not self.get_rgbd_data():
            return "aborted"

        userdata.image = self.rgb_image
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
                             input_keys=['image'],
                             output_keys=['main_face'])

        self.face_detection_client = benpy.ServiceProxy('/bender/vision/face_detector/detect_face',FaceInfo)
        self.min_area = 100*100
        self.bridge = CvBridge()


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

        det_srv = FaceInfoRequest()
        det_srv.use_image = True
        det_srv.image     = userdata.image
        det_srv.return_images = True

        # try:
        #     rgb_image = self.bridge.imgmsg_to_cv2(userdata.image, "bgr8")
        #     cv2.imshow("rgb",rgb_image)
        #     cv2.waitKey(0)
        # except Exception, e:
        #     rospy.logerr("failed to convert ROS image to OpenCV image. why?: " + str(e))
        #     return 'aborted'
        try:
            response = self.face_detection_client(det_srv)

            if response.n_faces > 0:
                print str(response.n_faces)+" n face"
                main_idx = self.get_main_face(response.BBoxes)
                print str(main_idx)+"main" 
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

        self.max_retry_times = 7
        self.current_retry_times = 0

    def execute(self, userdata):
        
        rospy.loginfo('Executing state: HANDLE_ZERO_FACES')

        self.current_retry_times = self.current_retry_times + 1
        if self.current_retry_times > self.max_retry_times:
            return 'aborted'

        Talk.getInstance("Please, look at my hat", 3)
        return 'succeeded'
    

class AddFace(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted'],
                             input_keys=['face_id','face_im'])

        self.face_add_client = benpy.ServiceProxy('/bender/vision/face_recognizer/add',FaceRecognition)

    def execute(self, userdata):
        rospy.loginfo('Executing state: ADD_FACE')
        
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
            Talk.getInstance("I'm ready, now I can remember your eyes", 4)
            FaceOrder.ChangeFace("1313")
            return 'done'


def getInstance():


    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['n_enroll_images','angle_rgbd'],
                            output_keys=['enroll_face_id'])

    sm.userdata.enroll_face_id = -1
    sm.userdata.sm_main_face = Image()
    sm.userdata.image = Image()

    with sm:
        
        smach.StateMachine.add('ANNOUNCE_START',
            TalkState.getInstance('i want to remember you, please look at my hat', 4),
            transitions = {'succeeded':'MOVE_RGBD'}
        )
        
        smach.StateMachine.add('MOVE_RGBD', MoveAsus.getInstance(),
            transitions={'succeeded':'GET_FREE_ID'},
            remapping={'deg_angle':'angle_rgbd'}
        )
        
        smach.StateMachine.add('GET_FREE_ID', GetFreeId(),
            transitions={'succeeded':'GET_IMAGE',
                        'aborted':'aborted'},
            remapping={'free_face_id':'enroll_face_id'}
        )
        
        smach.StateMachine.add('GET_IMAGE',SubsImage(),
            transitions = {'succeeded':'DETECT_FACES'},
        )
        
        smach.StateMachine.add('DETECT_FACES',DetectFaces(),
            transitions={'succeeded':'ADD_FACE',
                         'aborted':'HANDLE_ZERO_FACES',
                         'srv_except':'aborted'},
            remapping={'main_face':'sm_main_face'}
        )

        smach.StateMachine.add('HANDLE_ZERO_FACES',HandleZeroFaces(),
            transitions={'succeeded':'GET_IMAGE',
                         'aborted':'aborted'}
        )

        smach.StateMachine.add('ADD_FACE',AddFace(),
            transitions={'succeeded':'HANDLE_ADD_FACE',
                         'aborted':'GET_IMAGE'},
            remapping={'face_id':'enroll_face_id',
                       'face_im':'sm_main_face'}
        )

        smach.StateMachine.add('HANDLE_ADD_FACE',HandleAddFace(),
            transitions={'done':'succeeded',
                         'need_more_imgs':'GET_IMAGE'},
            remapping={'n_enroll_images':'n_enroll_images'}
        )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('enroll_face_rgbd')

    sm = getInstance()
    ud = smach.UserData()
    ud.n_enroll_images = 10

    # introspection server
    sis = smach_ros.IntrospectionServer('enroll_face_rgbd', sm, '/ENROLL_FACE_RGBD_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
