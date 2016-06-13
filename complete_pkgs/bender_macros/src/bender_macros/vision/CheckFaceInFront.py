#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import tf

from math import sqrt
from math import pi
from math import tan

from bender_msgs.msg import Emotion
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import ValidPoint
from bender_srvs.srv import Transformer
from geometry_msgs.msg import PoseStamped

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class RotateHead(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','last_rotation','aborted','preempted'],
            input_keys = ['machine_state'] 
        )

        self.head_angles = [0, 20, -20]
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
        self.current_head_pos_index = 0
        self.last_head_angle = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state: ROTATE_HEAD')

        if userdata.machine_state == 'working':

            if self.current_head_pos_index >= len(self.head_angles):
                self.current_head_pos_index = 0
                self.rotateHead(0)

                return 'aborted'


            self.rotateHead(self.head_angles[self.current_head_pos_index])
            self.current_head_pos_index += 1

            return 'succeeded'

        else:
            self.rotateHead(0)

            return 'last_rotation'


    def rotateHead(self,angle):

        emotion = Emotion()
        emotion.Order = "MoveX"
        emotion.X = angle

        delta = abs(angle - self.last_head_angle)
        self.last_head_angle = angle

        rospy.loginfo("Rotating Head to " + str(angle) + " degrees")

        self.face_pub.publish(emotion)

        rospy.sleep(delta*3.5/40.0)

        

class CheckPose(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','invalid_pose','aborted','preempted'],
            input_keys = ['face_data'],
            output_keys = ['face_pose', 'machine_state']
        )

        self.distance_th = rospy.get_param('near_person_distance_th', 1.6)
        self.cam_name = rospy.get_param('cam_name','camera_right_eye')
        self.cam_fov_h = rospy.get_param('/bender/sensors/' + self.cam_name + '/fov_h',60.0)
        self.cam_width = rospy.get_param('/bender/sensors/' + self.cam_name + '/width',640)
        self.cam_fov_h *= pi/180.0

        # camera model: y = a1*x^n + a2*x^(n-1) + ... + an
        default_cam_model = [
            5.5966e-22, -1.3651e-18, 1.4468e-15,
            -8.7314e-13, 3.3058e-10, -8.1623e-08,
            1.3248e-05, -1.3913e-03, 9.0455e-02,
            -3.3137e+00, 5.4756e+01
        ]
        self.cam_model = rospy.get_param('/bender/sensors/' + self.cam_name + '/face_model',default_cam_model)
        self.transform_client = rospy.ServiceProxy('/bender/tf/simple_pose_transformer/transform',Transformer)
        self.mappoint_client = rospy.ServiceProxy('/bender/nav/map_analyzer/check_point_inside_map',ValidPoint)


    def execute(self, userdata):

        rospy.loginfo('Executing state: CHECK_POSE')

        cam_frame = "/bender/sensors/" + self.cam_name + "_link"

        # input data
        face_width = userdata.face_data['width']
        face_offset = userdata.face_data['offset']

        # estimate face depth
        depth = 0
        power = len(self.cam_model) - 1
        for coef in self.cam_model:
            depth += coef*pow(face_width,power)
            power -= 1

        # estimated image resolution (m/pixel) 
        resolution = 2.0*depth*tan(self.cam_fov_h/2.0)/self.cam_width

        # estimated face pose
        face = PoseStamped()
        face.header.stamp = rospy.Time.now()
        face.header.frame_id = cam_frame
        face.pose.position.x = depth
        face.pose.position.y = -resolution*(
                face_offset + face_width - self.cam_width/2.0
        )
        face.pose.position.z = 0.0
        face.pose.orientation.w = 1.0
        

        # transform point to bender frame
        while (not rospy.is_shutdown()):
            
            try:
                self.transform_client.wait_for_service()
                transform_res = self.transform_client(pose_in=face, frame_out='/bender/base_link')
                break
                
            except Exception as e:
                print 'An error occurred while trying to transform the face pose:', e
                continue
                        
        trans_face = transform_res.pose_out

        
        # check if the point lies inside the valid Map        
        try:
            self.mappoint_client.wait_for_service()
            check_res = self.mappoint_client(point=trans_face.pose.position, frame_id='/bender/base_link')
                
        except Exception as e:
            print 'An error occurred while trying to transform the face pose:', e
            userdata.machine_state = 'working'
            return 'invalid_pose'


        if check_res.is_valid == False:
            userdata.machine_state = 'working'
            return 'invalid_pose'


        # check distance from person to bender
        fx = trans_face.pose.position.x
        fy = trans_face.pose.position.y
        distance = sqrt(fx*fx + fy*fy)
        if distance > self.distance_th:
            userdata.machine_state = 'working'
            return 'invalid_pose'            

        # return data
        rospy.loginfo('Person found at ' + str(distance) + ' [m] from bender')
        userdata.face_pose = trans_face

        return 'succeeded'


class DetectFace(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted','retry'],
            output_keys = ['face_data','machine_state']
        )

        self.face_detection_client = rospy.ServiceProxy(
            '/bender/vision/face_detector/detect_face',FaceInfo
        )


    def execute(self, userdata):

        rospy.loginfo('Executing state: DETECT_FACES')

        # face detection service
        self.face_detection_client.wait_for_service()
        try:
            response = self.face_detection_client(return_images = True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        # Analyze response
        rospy.loginfo('number of detected faces: ' + str(response.n_faces))
        if response.n_faces > 0:

            # Determine the biggest face
            max_width = 0
            face_index = 0
            max_face_index = 0
            for box in response.BBoxes:
                if box.width > max_width:
                    max_width = box.width
                    max_face_index = face_index
                face_index += 1


            # fill userdata
            userdata.face_data = {
                'width':max_width,
                'offset':response.BBoxes[max_face_index].x
            }
            userdata.machine_state = 'face_detected'

            return 'succeeded'

        else:
            rospy.sleep(1)
            return 'retry'

        return


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():


    # succeeded: face found
    # aborted: failed or face not found
    sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'],
        output_keys=['face_pose']
    )

    sm.userdata.face_pose = PoseStamped()
    sm.userdata.sm_machine_state = 'working'

    with sm:

        smach.StateMachine.add('ROTATE_HEAD',RotateHead(),
            transitions={
                'succeeded':'DETECT_FACES',
                'last_rotation':'succeeded'
            },
            remapping = { 'machine_state':'sm_machine_state' }
        )

        smach.StateMachine.add('DETECT_FACES',DetectFace(),
            transitions = {
                'succeeded':'CHECK_POSE',
                'retry':'ROTATE_HEAD'
            },
            remapping = { 
                'face_data':'sm_face_data',
                'machine_state':'sm_machine_state'
            }
        )

        smach.StateMachine.add('CHECK_POSE',CheckPose(),
            transitions = { 
                'succeeded':'ROTATE_HEAD',
                'invalid_pose':'ROTATE_HEAD'
            },
            remapping = {
                'face_data':'sm_face_data',
                'face_pose':'face_pose',
                'machine_state':'sm_machine_state'
            }
        )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('look_for_some_face')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('look_for_some_face', sm, '/LOOK_FOR_SOME_FACE_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
