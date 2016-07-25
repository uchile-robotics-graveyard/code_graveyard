#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
import math
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_utils.ros import benpy

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist

class moveBase(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['d'])
        rospy.Subscriber('/bender/nav/robot_pose_publisher/pose',PoseStamped,self.update)
        self.pub = rospy.Publisher('/bender/nav/base/cmd_vel', Twist, queue_size=1)
        self.pose = Pose()

    def update(self, data):
        self.pose = data.pose
        #print "poseee"
        #print self.pose

    def execute(self, userdata):
        
        order = Twist()
        
        if userdata.d > 0:
            order.linear.x = 0.1
        elif userdata.d < 0:
            order.linear.x = -0.1
        
        rospy.sleep(1)
        initial_pose = self.pose
        print initial_pose
        
        while True:
            self.pub.publish(order)
            distancia = math.sqrt(math.pow(self.pose.position.x-initial_pose.position.x,2)
             + math.pow(self.pose.position.y-initial_pose.position.y,2))
            print "move x : distancia: " + str(distancia) + "/" +str(userdata.d) + ", order x: " + str(order.linear.x)
            rospy.sleep(0.01)
            if math.fabs(distancia) > math.fabs(userdata.d):
                break
        
        # - - - - - frenar - - - - - -
        print "MOVE X: frenando"
        order.linear.x = 0.0
        self.pub.publish(order)
        rospy.sleep(0.1)
        
        return 'succeeded'


def getState():
    
    return moveBase()

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                            input_keys=['d'])
     
    with sm:
        smach.StateMachine.add('MOVE_BASE_X', getState(),
               transitions={'succeeded':'succeeded'},
               remapping={'d':'d'}
        )
    #sm.set_initial_state(['MOVE_BACK'])
    return sm

# main
if __name__ == '__main__':

  rospy.init_node('move_base_x_sm')

  sm = getInstance()

  ud = smach.UserData()
  ud.d = 1.5
  
  # introspection server
  sis = smach_ros.IntrospectionServer('move_base_x_sm', sm, '/MOVE_X_BASE_SM')
  sis.start()
  outcome = sm.execute(ud)
  sis.stop()