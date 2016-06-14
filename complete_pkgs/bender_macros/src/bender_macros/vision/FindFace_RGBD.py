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
from bender_macros.head import MoveAsus


#  - - - - macros - - - -
from bender_macros.speech import Talk
from bender_macros.head import FaceOrder
import sensor_msgs


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #



class SubsImage(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
                                output_keys=['image'])
        
                
        self.rgb_subs   = None
        
        self.rgb_image = Image()
        self.bridge = CvBridge()
        self.cv_rgb_image = []
        self.cv_depth_image = []
        self.got_rgb_img = False
        self.got_images = False

    def rgb_cb(self,msg):
        self.rgb_image = msg
        self.got_rgb_img = True

    def get_rgbd_data(self):
        
        rgb_topic   = '/bender/sensors/rgbd_head/rgb/image_raw'
        self.rgb_subs = rospy.Subscriber(rgb_topic, sensor_msgs.msg.Image, self.rgb_cb)

        if not self.got_images:
            
            while not (self.got_rgb_img):
                rospy.sleep(0.1)

            self.got_images = True

        self.rgb_subs.unregister()

        return True

    def execute(self, userdata):
        self.got_rgb_img = False
        self.got_images = False
        if not self.get_rgbd_data():
            return "preempted"

        userdata.image = self.rgb_image

        return 'succeeded'



class DetectFaces(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Face Detected','Face Not Detected','succeeded','aborted'],
                             input_keys=['image'],
                             output_keys=['main_face'])

        self.face_detection_client = benpy.ServiceProxy('/bender/vision/face_detector/detect_face',FaceInfo)
        self.min_area = 100*100
        self.bridge = CvBridge()

        self.counterface = 0
        self.counternoface = 0

        self.count = 1

    def get_main_face(self, bboxes):
        
        max_area = 0
        max_idx  = -1
        print len(bboxes)
        for k in xrange(len(bboxes)):
            
            box = bboxes[k]
            area = box.width*box.height
            
            rospy.logwarn("bbox w,h = " + str(box.width) + ", " + str(box.height))

            if area > max_area :
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

        try:
            response = self.face_detection_client(det_srv)
            entro = False
            if response.n_faces > 0:
                print str(response.n_faces)+" n face"
                main_idx = self.get_main_face(response.BBoxes)

                if main_idx >= 0:
                    self.counterface+=1
                    self.counternoface=0
                    entro = True
                
                userdata.main_face = response.faces[main_idx]
                rospy.loginfo('number of detected faces: ' + str(response.n_faces))
            if entro == False:
                self.counterface=0
                self.counternoface+=1

            if self.counterface>=2:
                self.counterface=0
                return 'Face Detected'

            if self.counternoface>=3:
                self.count+=1
                self.counternoface=0
                return 'Face Not Detected'

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return 'succeeded'



class SelectRGBDAngle(smach.State):

   def __init__(self, rgbd_angles):
      smach.State.__init__(self, 
                           outcomes=['fail','succeeded','aborted','preempted'],
                           output_keys=['selected'])
      # angles
      self.angles = rgbd_angles
      self.ang_id = -1

   def execute(self, userdata):
      rospy.loginfo('Executing state SelectRGBDAngle')

      self.ang_id+=1

      if self.ang_id >= len(self.angles):
         self.ang_id = -1
         return 'fail'

      nang = self.ang_id%len(self.angles)
      userdata.selected = self.angles[nang]
      rospy.loginfo('setting rgbd angle> ' + str(self.angles[nang]))

      return 'succeeded'


def getInstance():


    sm = smach.StateMachine(outcomes=['Face Detected','Face Not Detected','succeeded','aborted','preempted'],
                    output_keys=['current_rgbd_angle'])

    sm.userdata.enroll_face_id = -1
    sm.userdata.sm_main_face = Image()
    sm.userdata.image = Image()

    sm.userdata.angle_face = [-10,0,25,35]
    sm.userdata.angle_facerecognition = 0

    rgbd_angles = [-10,0,25,35]
    sm.userdata.current_rgbd_angle = 0

    with sm:
        smach.StateMachine.add('MOVE_ASUS',MoveAsus.getInstance(),
                transitions={'succeeded':'SUBS_IMAGE'},
                remapping={'deg_angle':'current_rgbd_angle'}
        )
        smach.StateMachine.add('SUBS_IMAGE',SubsImage(),
                transitions = {'succeeded':'DETECT_FACES'},
            )
        smach.StateMachine.add('DETECT_FACES',DetectFaces(),
                           transitions={'Face Detected': 'Face Detected', 
                           'Face Not Detected': 'SELECT_RGBD_ANGLE', 
                                    'succeeded':'DETECT_FACES',
                                    'aborted':'DETECT_FACES'},
                           remapping={'main_face':'sm_main_face'})
        ## scan looking for a shelf
        smach.StateMachine.add('SELECT_RGBD_ANGLE', SelectRGBDAngle(rgbd_angles),
            transitions={'succeeded':'MOVE_ASUS',
                         'fail':'Face Not Detected'},
            remapping={'selected':'current_rgbd_angle'}
        )
    return sm


# main
if __name__ == '__main__':

    rospy.init_node('find_face_rgbd')

    sm = getInstance()
    ud = smach.UserData()

    # introspection server
    sis = smach_ros.IntrospectionServer('find_face_rgbd', sm, '/FIND_FACE_RGBD_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
