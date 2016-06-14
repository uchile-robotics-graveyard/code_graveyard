#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped

#  - - - - macros - - - -
from bender_macros.nav import LookToPoseStamped
from bender_macros.nav import ApproachToPoseStamped

# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #
# class ManageCrowd(smach.State):

#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'],
#                              input_keys=['pose_approach','pose_look'],
#                              output_keys=['approach','look'],
#                              )
#         self.contador = 0

#     def execute(self,userdata):

#         rospy.loginfo('Executing State ManageCrowd for the crowd ' + str(self.contador+1))

#         if len(userdata.pose_approach)>1 :
#             rospy.loginfo('Crowds: '+str(len(userdata.pose_approach))+' --> Crowd Size: '+str(len(userdata.pose_look[self.contador])))
#             if self.contador == 0:
#                 userdata.approach = userdata.pose_approach[0]
#                 userdata.look = userdata.pose_look[0]
#             elif self.contador == 1:
#                 userdata.approach = userdata.pose_approach[1]
#                 userdata.look = userdata.pose_look[1]
#         elif len(userdata.pose_approach)==1:
#             userdata.approach = userdata.pose_approach[0]
#             userdata.look = userdata.pose_look[0] 
#         else:
#             return 'aborted'    

#         print userdata.pose_approach,userdata.pose_look   
               
#         self.contador += 1
#         return 'succeeded'

def getInstance():
    
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['pose_approach','pose_look'])

    with sm:

        # smach.StateMachine.add('MANAGE_CROWDS', ManageCrowd(),
        #     transitions = {'succeeded':'APPROACH_TO_CROWD',
        #                     'aborted':'aborted'},
        #     remapping = {'pose_approach':'pose_approach','pose_look':'pose_look',
        #                 'approach':'approach','look':'look'}
        # )
        smach.StateMachine.add('APPROACH_TO_CROWD', ApproachToPoseStamped.getInstance(),
            transitions = {'succeeded':'LOOK_PERSON',
                            'aborted':'aborted'},
            remapping = {'goal_pose':'pose_approach'}
        )

        smach.StateMachine.add('LOOK_PERSON', LookToPoseStamped.getInstance(),
            transitions = {'succeeded':'succeeded'},
            remapping = {'goal_pose':'pose_look'}
        )

    return sm

# main
if __name__ == '__main__':

    rospy.init_node('approach_look')

    sm = getInstance()
    
    # userdata
    ud = smach.UserData()
    
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.orientation.w = 1.0
    ps.pose.position.x = 0.0
    ps.pose.position.y = 0.0
    ud.pose_approach = ps

    psc = PoseStamped()
    psc.header.frame_id = "map"
    psc.header.stamp = rospy.Time.now()
    psc.pose.position.x =  1.0
    psc.pose.position.y = -1.0
    psc.pose.orientation.w = 1.0
    ud.pose_look = psc

    # introspection server
    sis = smach_ros.IntrospectionServer('approach_look', sm, '/APPROACH_LOOK_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
