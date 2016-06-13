#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from math import sqrt
from math import pi
from bender_utils.ros import benpy

from bender_arm_control.srv import HeadPosition, HeadPositionRequest, HeadPositionResponse
from sensor_msgs.msg import JointState

class moverAsus(smach.State):

    def __init__(self, blocking=True):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                                    input_keys=['deg_angle'])

        # move server
        self.rgbd_position = rospy.ServiceProxy('/bender/rgbd_position', HeadPosition)

        # position control
        self.blocking = blocking
        self.position_sub = None
        self.current_theta = 0
        self.valid_theta = False
        
    def execute(self, userdata):
        rospy.loginfo('Executing state MoveAsus, angle:' + str(userdata.deg_angle))

        # positivo hacia abajo
        theta = userdata.deg_angle*pi/180
        theta = min(theta, 1.15) # min tilt!

        req = HeadPositionRequest()
        req.position.data = theta
        try:
            resp = self.rgbd_position(req)
            # TODO: THIS DOESNT WORK AT ALL!, 
            # SO A HAD TO IMPLEMENT THE PATCH WITH wait_result()
            #if resp.reach:
            #    return 'succeeded'
            #else:
            #    return 'aborted'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'aborted'

        if self.blocking:
            self.wait_result(theta)

        return 'succeeded'

    def wait_result(self, position):

        # subscribe
        self.position_sub = rospy.Subscriber('/bender/head/joint_states', JointState, self.rgbd_position_cb)

        # wait valid msgs
        while not self.valid_theta:
            rospy.sleep(0.1)

        # correction
        while abs(self.current_theta - position) > 0.017:
            rospy.sleep(0.1)

        # ready
        self.position_sub.unregister()
        self.valid_theta = False

    def rgbd_position_cb(self, msg):
        # update position
        self.current_theta = msg.position[0]
        self.valid_theta = True

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def getReadyMachine(angle):
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm.userdata.deg_angle = angle
    
    with sm:

        smach.StateMachine.add('MOVE_RGBD',moverAsus(),
                transitions={'succeeded':'succeeded'},
                remapping={'deg_angle':'deg_angle'}
        )

    return sm

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def getFrontFaceAnalysisInstance():
    # angulo par ver cara de persona que esta al frente ~1m
    return getReadyMachine(20)

def getNavigationInstance():
    # angulo muy importante para navegar c:
    return getReadyMachine(40)

def getTableApproachInstance():
    # very important angle!. By using a lower one
    # the robot would hit the table/shelf 
    # for this macro to work (using 60[deg]. table should not be farther than ~1[m]), 
    # todo: inclinar sensor, segun la distancia a la mesa
    return getReadyMachine(50)

def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['deg_angle'])

    with sm:

        smach.StateMachine.add('MOVE_RGBD',moverAsus(),
                transitions={'succeeded':'succeeded'},
                remapping={'deg_angle':'deg_angle'})

    return sm
 
 
# main
if __name__ == '__main__':

    rospy.init_node('MoveAsus')

    # nav> 40
    # table approach> 50
    # object perception table> ?
    sm = getReadyMachine(40)

    # introspection server
    sis = smach_ros.IntrospectionServer('MOVE_RGBD', sm, '/MOVE_RGBD_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
