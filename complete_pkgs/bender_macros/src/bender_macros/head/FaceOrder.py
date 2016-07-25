#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *

def ChangeFace(order):

    face_pub = rospy.Publisher('/bender/face/head', bender_msgs.msg.Emotion, queue_size=1)
    rospy.sleep(0.1)
    emo = bender_msgs.msg.Emotion()

    if type(order)==str:
        emo.Order = 'changeFace'
        emo.Action = str(order)
        face_pub.publish(emo)
        rospy.loginfo('Setting face to: '+order)
        return True

    elif type(order)==int:
        emo.Order = 'MoveX'
        emo.X = order
        face_pub.publish(emo)
        rospy.loginfo('Moving head to: ' + str(order))
        return True

    else:
        rospy.logwarn('Please give a string or integer')
        return False


class ChangeFaceState(smach.State):

  def __init__(self,order):
    smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
    self.order = order
    
  def execute(self, userdata):
    rospy.loginfo('Executing state ChangeFaceState')
    if ChangeFace(order):
        return 'succeeded'
    else:
        return 'aborted'


def getInstance(order):
    return ChangeFaceState(order)


if __name__ == '__main__':

    rospy.init_node('FaceTest')
    
    # sm = getInstance(30)
    
    # sis = smach_ros.IntrospectionServer('FaceTest', sm, '/FACE_TEST')
    # sis.start()
    # outcome = sm.execute()
    # sis.stop()

    ChangeFace(10)