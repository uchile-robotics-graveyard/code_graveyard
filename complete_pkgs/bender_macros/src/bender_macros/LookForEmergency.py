#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros
import time
import tf

from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from bender_msgs.msg import Emotion
from bender_srvs.srv import FaceInfo
from bender_srvs.srv import Transformer

from bender_srvs.srv import DetectEmerg
from bender_srvs.srv import synthesize
from bender_srvs.srv import ImageService
from bender_srvs.srv import ImageServiceRequest
from bender_srvs.srv import ImageServiceResponse
from bender_srvs.srv import SearchPerson

# solo para testeo
import cv2
import cv_bridge
import numpy as np

hubo_timeout = False

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            output_keys = ['machine_state','images', 'image_captions'],
        )

    def execute(self, userdata):
        
        # reset machine variables
        userdata.machine_state = 'working'
        userdata.images = []
        userdata.image_captions = []
        
        # turn off camera right eye
        img_turn_off_client = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_off', Empty)
        img_turn_off_client.wait_for_service()
        img_turn_off_client()
        
        # turn on camera thermal
        thermal_turn_on_client = rospy.ServiceProxy('/bender/sensors/camera_thermal/turn_on', Empty)
        thermal_turn_on_client.wait_for_service()
        thermal_turn_on_client()

        return 'succeeded'

class LastSetup(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes = ['succeeded','aborted','preempted','timeout'])

    def execute(self, userdata):
        global hubo_timeout

        # turn off camera thermal
        thermal_turn_off_client = rospy.ServiceProxy('/bender/sensors/camera_thermal/turn_off', Empty)
        thermal_turn_off_client.wait_for_service()
        thermal_turn_off_client()

        if hubo_timeout:
            return 'timeout'
        
        return 'succeeded'
        
class RotateHead(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','last_rotation','aborted','preempted'],
            input_keys = ['machine_state'] 
        )

        self.head_angles = [0, 35, -35, 0]
        self.face_pub = rospy.Publisher('/bender/face/head', Emotion)
        self.current_head_pos_index = 0
        self.last_head_angle = 0

    def execute(self, userdata):
        global hubo_timeout 

        rospy.loginfo('Executing state: ROTATE_HEAD')

        if hubo_timeout:
            self.current_head_pos_index = 0
            self.rotateHead(0)
            
            return 'last_rotation'

        if userdata.machine_state == 'working':

            if self.current_head_pos_index >= len(self.head_angles):
                self.current_head_pos_index = 0
                self.rotateHead(0)

                return 'aborted'

            self.rotateHead(self.head_angles[self.current_head_pos_index])
            self.current_head_pos_index += 1

            return 'succeeded'
        else:
            self.current_head_pos_index = 0
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

class DetectEmergency(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded','timeout','person_fine','aborted','preempted'],
            output_keys = ['emergency_data','machine_state'],
            io_keys = ['images', 'image_captions']
        )
        
        self.thermal_turn_on_client = rospy.ServiceProxy('/bender/sensors/camera_thermal/turn_on', Empty)
        self.thermal_turn_off_client = rospy.ServiceProxy('/bender/sensors/camera_thermal/turn_off', Empty)
        self.img_turn_on_client = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_on', Empty)
        self.img_turn_off_client = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_off', Empty)
        self.img_client = rospy.ServiceProxy('/bender/sensors/camera_right_eye/get_image', ImageService)
        self.emergency_detector_client = rospy.ServiceProxy('/bender/vision/emergency_detector/detect_emergency', DetectEmerg) 
        self.wave_client = rospy.ServiceProxy('/search_waving/search',SearchPerson)

        self.last_stand_distance = -1
        self.taken_stand_photo = False
        self.initial_time = -1
        self.initial_real_time = -1
        self.previous_state = "none"
    
    def talk(self, text, sleep_time=0):
        
        # talk
        talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        talk_client.wait_for_service()
        talk_client(text)
        
        # sleep
        rospy.sleep(sleep_time)
        
    def take_photo(self, ud, info):
        
        userdata = self.take_photo_thermal(userdata,'Still no emergency: standing position')

        # turn off camera thermal
        self.thermal_turn_off_client.wait_for_service()
        self.thermal_turn_off_client()
        
        # turn on camera right eye
        self.img_turn_on_client.wait_for_service()
        self.img_turn_on_client()
        
        # take picture
        self.img_client.wait_for_service()            
        img_res = self.img_client()
        ud.images.append(img_res.im)

        # picture data
        curr_time = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
        caption = info + '. Taken at time: ' + curr_time
        ud.image_captions.append(caption)

        # turn off camera right eye
        self.img_turn_off_client.wait_for_service()
        self.img_turn_off_client()
        
        # turn on camera thermal
        self.thermal_turn_on_client.wait_for_service()
        self.thermal_turn_on_client()
                
        return ud
    
        
    def take_photo_thermal(self, ud, info):
          
        # take picture
        self.img_client.wait_for_service()            
        img_res = self.img_client()
        ud.images.append(img_res.im)

        # picture data
        curr_time = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
        caption = info + '. Taken at time: ' + curr_time
        ud.image_captions.append(caption)
                
        return ud

    def execute(self, userdata):

        global hubo_timeout

        if self.initial_real_time < 0:
            self.initial_real_time = time.time()

        # the first execution time
        if self.initial_time < 0:
            self.initial_time = time.time()
            self.taken_stand_photo = False
            self.last_stand_distance = -1
            self.previous_state = "none"
            
            # turn off camera right eye
            self.img_turn_off_client.wait_for_service()
            self.img_turn_off_client()
            
            # turn on camera thermal
            self.thermal_turn_on_client.wait_for_service()
            self.thermal_turn_on_client()
        
        while True:
    
            dtime = time.time() - self.initial_real_time
       	  
            if dtime > 55:
                self.talk("I not found someone, please come here") 
                rospy.sleep(5)

                for i in range(4): 
                    resp = self.emergency_detector_client()
                    
                    if not resp.state == "notfound" and not self.taken_stand_photo:

                        userdata = self.take_photo(userdata,'Still no emergency: standing position')
                        rospy.sleep(1)

                    bool1 = (resp.state == "notfound")
                    if not bool1 and not self.taken_stand_photo:
                            
                        userdata = self.take_photo(userdata,'Emergency not found')
                        rospy.sleep(1)

                userdata.emergency_data = 0
                userdata.machine_state = 'emergency_detected'
                hubo_timeout = True

                return "timeout"

            # will try for 5 seconds
            dtime = time.time() - self.initial_time 
            if dtime > 5:
                
                if self.previous_state == "stand" and dtime < 10:
                    # if we found the person, then wait at max 10[s] for emergency
                    pass
                else:
                    self.initial_time = -1
                    return "aborted"
    
    
            resp = self.emergency_detector_client()
            abc =resp.distance
            if resp.distance> 3.5: 
                abc=self.last_stand_distance

            # check standing position
            if resp.state == "stand":
                
                self.previous_state = "stand"
                self.last_stand_distance = abc
                userdata.emergency_data = abc
                
                if not self.taken_stand_photo:
                    
                    # announce & take photo
                    rospy.loginfo("person found: stand pose")
                    self.talk("I found someone")         
                    userdata = self.take_photo(userdata,'Still no emergency: standing position')
                    rospy.sleep(1)
                    
                    self.taken_stand_photo = True
                
            # check seating position
            if resp.state == "seat":
                
                # announce & take photo
                rospy.loginfo("emergency found: person seated")
                self.talk("I found an emergency")
                userdata = self.take_photo(userdata, 'Emergency: "seat"')
                rospy.sleep(1)
                
                # fill data
                userdata.emergency_data = abc
                userdata.machine_state = 'emergency_detected'
                
                # prepare for future usage
                self.initial_time = -1
                
                return "succeeded"
            
            # check standing to disappearing transition  
            if resp.state == "notfound" and self.previous_state == "stand":
                
                # announce & take photo
                rospy.loginfo("emergency found: walking -> disappeared")
                self.talk("I found an emergency")
                userdata = self.take_photo(userdata,'Emergency: person on the floor')
                rospy.sleep(1)
                
                # fill data
                userdata.emergency_data = self.last_stand_distance
                userdata.machine_state = 'emergency_detected'
                
                # prepare for future usage
                self.initial_time = -1
                
                return "succeeded"

                #  res=self.wave_client()
                # if res.found==True:     
                #    userdata.emergency_data = self.last_stand_distance
                #   userdata.machine_state = 'emergency_detected'       
                 #  return "succeeded"

            rospy.sleep(0.3)
            return 'person_fine'
            
class ExtractPose(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['emergency_data'],
            output_keys = ['person_pose']
        )
     
        self.transform_client = rospy.ServiceProxy(
            '/bender/tf/simple_pose_transformer/transform',Transformer
        )


    def execute(self, userdata):

        rospy.loginfo('Executing state: EXTRACT_POSE')

        # OBS: this should be: .../camera_thermal_link!!, but the the difference is small
        cam_frame = "/bender/sensors/camera_right_eye_link"

        # estimated face pose
        person = PoseStamped()
        person.header.stamp = rospy.Time.now()
        person.header.frame_id = cam_frame
        person.pose.position.x = userdata.emergency_data
        person.pose.orientation.w = 1.0
        
        
        while (not rospy.is_shutdown()):
            
            try:
                self.transform_client.wait_for_service()
                transform_res = self.transform_client(pose_in=person, frame_out='/map')
                break
                
            except Exception as e:
                print 'An error occurred while trying to transform the face pose:', e
                continue

        # return data                        
        userdata.person_pose = transform_res.pose_out

        return 'succeeded'


# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    M a c h i n e                           #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #

def getInstance():

    sm = smach.StateMachine(
        outcomes = ['succeeded','aborted','preempted','timeout'],
        output_keys = ['person_pose','images', 'image_captions'],
        input_keys = ['images', 'image_captions','map_name']
    )
    
    sm.userdata.person_pose = PoseStamped()
    sm.userdata.machine_state = 'working'
    sm.userdata.images = []
    sm.userdata.image_captions = []

    with sm:

        smach.StateMachine.add('SETUP',Setup(),
            transitions={'succeeded':'ROTATE_HEAD'},
            remapping = { 'machine_state':'machine_state',
                          'images':'images',
                          'image_captions':'image_captions'}
        )

        smach.StateMachine.add('ROTATE_HEAD',RotateHead(),
            transitions={
                'succeeded':'DETECT_EMERGENCY',
                'last_rotation':'LAST_SETUP',
                'aborted':'aborted'
            },
            remapping = { 'machine_state':'machine_state' }
        )

        smach.StateMachine.add('DETECT_EMERGENCY', DetectEmergency(),
            transitions = {'succeeded':'EXTRACT_POSE',
                           'person_fine':'DETECT_EMERGENCY',
                           'timeout':'EXTRACT_POSE',
			   'aborted':'ROTATE_HEAD'
            },
            remapping = {'emergency_data':'emergency_data',
                         'images':'images',
                         'image_captions':'image_captions',
                         'machine_state':'machine_state'
            }
        )
        
        smach.StateMachine.add('EXTRACT_POSE',ExtractPose(),
            transitions = { 'succeeded':'ROTATE_HEAD' },
            remapping = {
                'emergency_data':'emergency_data',
                'person_pose':'person_pose'
            }
        )
        
        smach.StateMachine.add('LAST_SETUP',LastSetup(),
            transitions={'succeeded':'succeeded',
                         'timeout':'timeout'}
        )

    return sm


# main
if __name__ == '__main__':

    rospy.init_node('look_for_emergency')

    sm = getInstance()
    
    # dummy userdata
    ud = smach.UserData()
    ud.map_name = 'amtc.sem_map'
    ud.images = []
    ud.image_captions = []

    # introspection server
    sis = smach_ros.IntrospectionServer('look_for_emergency', sm, '/LOOK_FOR_EMERGENCY')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
