#!/usr/bin/env python

import roslib
roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
from bender_macros.speech import Talk

import cv2
from bender_srvs.srv import ImageService, IsOn, play_sound
from cv_bridge import CvBridge, CvBridgeError
from bender_macros.head import FaceOrder 
from std_srvs.srv import Empty

class SelfieState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])

        self._getImage = rospy.ServiceProxy('/bender/sensors/camera/get_image',ImageService)
        self._isOn = rospy.ServiceProxy('/bender/sensors/camera/is_on',IsOn)
        self._turnOn = rospy.ServiceProxy('/bender/sensors/camera/turn_on',Empty)
        self._turnOff = rospy.ServiceProxy('/bender/sensors/camera/turn_off',Empty)
        self._asturnOn = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_on',Empty)
        self._asturnOff = rospy.ServiceProxy('/bender/sensors/camera_right_eye/turn_off',Empty)
        self._sound = rospy.ServiceProxy('/bender/fun/sound_player/play',play_sound)
        self.count = 0

    def execute(self,userdata):

        self._asturnOff()
        self._turnOn()
        self.commandHead(30)
        rospy.sleep(2)
        self.commandHead('happy2')
        rospy.sleep(2)
        self.takePhoto()
        self.commandHead(0)
        rospy.sleep(2)
        #Talk.getInstance("now you look pretty! ",3)
        self._turnOff()
        self._asturnOn()
        return 'succeeded'

    def takePhoto(self):

        self.bridge = CvBridge()

        try:
            im = self._getImage('')
            cv_image = self.bridge.imgmsg_to_cv2(im.im, "bgr8")
        except CvBridgeError, e:
            print e

        self._sound( 1, 'photo_sound')
        cv2.imwrite('/home/bender/selfie'+str(self.count)+'.jpg',cv_image)
        cv2.imshow("Selfie", cv_image)
        cv2.waitKey(5000)
        self.count+=1
        #rospy.sleep(5)

    def commandHead(self,command):

        FaceOrder.ChangeFace(command)

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
        smach.StateMachine.add('SELFIE', SelfieState())
    return sm

if __name__ == '__main__':
    
    rospy.init_node('selfie') 

    sm = getInstance()
    
    sis = smach_ros.IntrospectionServer('selfie', sm, '/SELFIE')
    sis.start()
    outcome = sm.execute()
    sis.stop()


