#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import math
import smach
import smach_ros

import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_core import benpy
from bender_macros.head import MoveAsus


class SearchOperator(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                input_keys=['crowd_size_hint'],
            output_keys=['operator_pose','operator_score'])
        
        # ROS interfaces
        self.face_detection_client   = benpy.ServiceProxy('/bender/vision/face_detector/detect_face', FaceInfo)
        self.face_recognition_client = benpy.ServiceProxy('/bender/vision/face_recognizer/recognize', FaceRecognition)
        self.transform_client = benpy.ServiceProxy('/bender/tf/simple_pose_transformer/transform', Transformer)
        self.rgb_topic   = '/bender/sensors/rgbd_head/rgb/image_raw'
        self.depth_topic = '/bender/sensors/rgbd_head/depth/image_raw'
        self.rgb_subs   = None
        self.depth_subs = None

        # ROS data   
        self.rgb_image   = None
        self.depth_image = None
        
        # ROS -> opencv data
        self.bridge = CvBridge()
        self.cv_rgb_image = []
        self.cv_depth_image = []
        
        # control
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False

        # start on "pause" state
        self.pause_dataflow()

    def init_dataflow(self):
        
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if not self.rgb_subs:
            self.rgb_subs   = rospy.Subscriber(self.rgb_topic, sensor_msgs.msg.Image, self.rgb_cb)
            self.depth_subs = rospy.Subscriber(self.depth_topic, sensor_msgs.msg.Image, self.depth_cb)
        
    def pause_dataflow(self):
        self.got_rgb_img = False
        self.got_depth_img = False
        self.got_images = False
        self.got_images_to_cv2 = False
        if self.rgb_subs:
            self.rgb_subs.unregister()
            self.depth_subs.unregister()
        self.rgb_subs   = None
        self.depth_subs = None
        
    def reset_images(self):
        self.got_images = False
        self.got_images_to_cv2 = False

    def rgb_cb(self,msg):
        self.rgb_image = msg
        self.got_rgb_img = True
        
    def depth_cb(self,msg):
        self.depth_image = msg
        self.got_depth_img = True
        
    def get_rgbd_data(self):
        
        self.init_dataflow()
        if not self.got_images:
            
            rospy.sleep(0.5)
            
            while not (self.got_rgb_img and self.got_depth_img):
                rospy.sleep(0.1)
                
            self.got_images = True

        if not self.got_images_to_cv2:
            try:
                self.cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
                self.cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, "16UC1")
            except rospy.ServiceException as e:
                rospy.logerr("SearchOperator: failed to convert ROS image to OpenCV image. why?: " + str(e))
                self.pause_dataflow()
                return False
            self.got_images_to_cv2 = True
        
        self.pause_dataflow()
        return True
        
    def transform_pose(self,person,frame):
        
        try:
            tf_req = TransformerRequest()
            tf_req.pose_in = person
            tf_req.frame_out = frame
            transf_out = self.transform_client(tf_req)
            
        except rospy.ServiceException as e:
            rospy.logwarn("SearchOperator: failed to transform face pose. why?: " + str(e))
            return False
            
        return transf_out.pose_out
        
    def get_position_from_bbox(self, bbox):
        """
        ESTA TESTEADO. Calculos son correctos
        """
        
        person = geometry_msgs.msg.PoseStamped()
        person.header = self.depth_image.header
        person.pose.orientation.w = 1.0
        
        # todo .. poner limites
        depth_w = self.depth_image.width
        depth_h = self.depth_image.height
        rgb_w = self.rgb_image.width
        rgb_h = self.rgb_image.height
        
        ax = int(round((bbox.x + bbox.width/2.0 )*(float(depth_w)/float(rgb_w)) ))
        by = int(round((bbox.y + bbox.height/2.0)*(float(depth_h)/float(rgb_h)) ))
        cons_pcl = 0.0019047619
        #print "depth w: " + str(depth_w)
        #print "depth y: " + str(depth_h)
        #print "rgb w: " + str(rgb_w)
        #print "rgb y: " + str(rgb_h)
        #print "box x: " + str((bbox.x + bbox.width/2.0 ))
        #print "box y " + str((bbox.y + bbox.height/2.0))
        #print "ax: " + str(ax)
        #print "by: " + str(by)
        
        depth = self.cv_depth_image[by,ax]/1000.0 # esta en [mm]
        depth = depth[0]
        px = depth*cons_pcl*(ax - depth_w/2)
        py = depth*cons_pcl*(by - depth_h/2)
        person.pose.position.x = px
        person.pose.position.y = py
        person.pose.position.z = depth
        rospy.logwarn("pose (cam_frame): (x,y,z)=(" + str(px) + "," + str(py) + "," + str(depth) + ")")
        
        #person_b = self.transform_pose(person, "/bender/base_link")
        person_m = self.transform_pose(person, "/map")
        #persons = [person_b, person_m]
        # Failed to transform!
        #if not (person_b and person_m):
        if not person_m:
            return False
        
        #         rospy.logwarn("pose (base_link frame): (x,y,z)=(" 
        #                       + str(person_b.pose.position.x) + ","
        #                       + str(person_b.pose.position.y) + ","
        #                       + str(person_b.pose.position.z) + ")")
        rospy.logwarn("pose (map frame): (x,y,z)=("
                      + str(person_m.pose.position.x) + ","
                      + str(person_m.pose.position.y) + "," 
                      + str(person_m.pose.position.z) + ")")
        
        return person_m
    
    def recognize_operator(self, face,n):
        
        rec_srv = FaceRecognitionRequest()
        rec_srv.use_image = True
        
        # bbox limits
        #x1 = max(bbox.x,0)
        #y1 = max(bbox.y,0)
        #x2 = min(x1 + bbox.width , self.rgb_image.width -1)
        #y2 = min(y1 + bbox.height, self.rgb_image.height-1)
        
        # crop bbox
        rec_srv.image = face #self.rgb_image[y1:y2,x1:x2] 
        
        try:
            
            result = FaceRecognitionResponse()
            result = self.face_recognition_client(rec_srv)
            #result = self.bestnface(result, n)
        except rospy.ServiceException as e:
            rospy.logwarn("SearchOperator: failed to detect faces on rgbd data. Aborting. why?: " + str(e))
            return 1000
        
        #cv2.imshow("operator cropped recognition dist: " + str(result.distance), rec_srv.image)
        #cv2.waitKey(1)
        
        return result.distance

    def bestnface(self, detection, n):

        faces = detection.faces
        boxes = detection.BBoxes
        eyes_positionL = detection.eyes_positionL
        eyes_positionR = detection.eyes_positionR

        faces_out = []
        boxes_out = []
        eyes_positionL_out = []
        eyes_positionR_out = []
        facessave = []

        for i in xrange(n):
            maxface = i
            k =0
            ksave=0
            for j in faces:
                if j.height*j.width>maxface.height*maxface.width and k not in facessave:
                    maxface = j
                    ksave=k
                k +=1
            if maxface.height*maxface.width > self.min_size_face and (maxface.height*maxface.width  - faces_out[min(i-1,0)].height*faces_out[min(i-1,0)].width)< self.distance_between_face:
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

    def execute(self, userdata): 
        
        if not self.get_rgbd_data():
            return "aborted"
        
        det_srv = FaceInfoRequest()
        det_srv.use_image = True
        det_srv.image = self.rgb_image
        det_srv.return_images = True
        
        try:
            det_resp = self.face_detection_client(det_srv)
            rospy.loginfo("found " + str(det_resp.n_faces) + " faces")
            
            if det_resp.n_faces <= 1:
                return "aborted"
            
        except rospy.ServiceException as e:
            rospy.logwarn("SearchOperator: failed to detect faces on rgbd data. Aborting. why?: " + str(e))
            return "aborted"
        
        # recognize the operator
        best_score = 1000
        best_idx   = 0
        for i in xrange(det_resp.n_faces):
        
            score = self.recognize_operator(det_resp.faces[i],userdata.crowd_size_hint)
            
            if score < best_score:
                best_score = score
                best_idx   = i
        
        rospy.loginfo('operator found: idx: ' + str(best_idx) + ', score: ' + str(best_score))
        
        # operator pose
        op_pose = self.get_position_from_bbox(det_resp.BBoxes[best_idx])
        if not op_pose:
            rospy.logwarn("SearchOperator: failed get operator pose from bbox. Aborting.")
            return "aborted"
        userdata.operator_pose  = op_pose
        userdata.operator_score = best_score
        
        return 'succeeded'
    
def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                    input_keys  = ['rgbd_angle','crowd_size_hint'],
                    output_keys = ['operator_pose', 'operator_score'])
    
    with sm:
    
        smach.StateMachine.add('MOVE_RGBD', MoveAsus.getInstance(),
            transitions={'succeeded':'SEARCH_OPERATOR'},
            remapping={'deg_angle':'rgbd_angle'}
        )
        
        smach.StateMachine.add('SEARCH_OPERATOR', SearchOperator(),
            transitions = {'succeeded':'succeeded',
                           'aborted':'aborted'},
            remapping = {'crowd_size_hint':'crowd_size_hint',
                         'operator_pose':'operator_pose',
                         'operator_score':'operator_score'}
        )
    
    
    
    return sm
    

if __name__ == '__main__':
    
    rospy.init_node('search_operator')
    
    sm = getInstance()
    ud = smach.UserData()
    ud.crowd_size_hint = 2
    ud.rgbd_angle = 15
    
    # introspection server
    sis = smach_ros.IntrospectionServer('search_operator', sm, '/SEARCH_OPERATOR_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
    