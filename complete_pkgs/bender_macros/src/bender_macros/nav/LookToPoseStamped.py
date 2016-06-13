#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from bender_macros.nav import GoalFromPoseStamped

def getInstance():
        
    # return goal_server/approach macro
    return GoalFromPoseStamped.getServiceInstance('look')
 
# main
if __name__ == '__main__':

    rospy.init_node('look_to_pose_stamped')

    sm = getInstance()

    # generate dummy goal_pose
    ud = smach.UserData()
    goal = PoseStamped()
    goal.header.frame_id = '/map'
    goal.header.stamp = rospy.Time.now()
    goal.pose.orientation.w = 1.0
    ud.goal_pose = goal

    # introspection server
    sis = smach_ros.IntrospectionServer('look_to_pose_stamped', sm, '/LOOK_TO_POSE_STAMPED_SM')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
 