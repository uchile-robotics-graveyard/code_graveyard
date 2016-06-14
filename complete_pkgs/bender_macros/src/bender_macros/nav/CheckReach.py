#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped 
from bender_srvs.srv import NavGoal
from bender_srvs.srv import NavGoalRequest
from bender_utils.ros import benpy

class CheckReach(smach.State):
    
    def __init__(self,timeout=0):
        
        smach.State.__init__(self, outcomes=['reached','aborted','preempted','not_reached'])

        # arrived client
        self.arrived_client = benpy.ServiceProxy('/bender/nav/goal_server/has_arrived', NavGoal)
        self.timeout = timeout
        self.elapsed_time = 0

    def execute(self, userdata):
        
        # 0: (GOAL_WAITING)  Waiting for next goal
        # 1: (GOAL_WALKING)  Moving toward goal
        # 2: (GOAL_ALMOST_REACHED) In rotation adjustment phase
        # 3: (GOAL_REACHED)  Reached OK.
        # 4: (GOAL_ABORTED)  Goal aborted by the server.
        # 5: (GOAL_CANCELED) Goal aborted by the user.
        
        try:
            
            # check arrive    
            self.arrived_client.wait_for_service()
            arrived_resp = self.arrived_client()

            while not rospy.is_shutdown():

                if arrived_resp.state == 4:
                    print 'Goal aborted by server'
                    return 'not_reached'

                elif arrived_resp.state == 5:
                    print 'Goal canceled by user'
                    return 'aborted'
                
                elif arrived_resp.state == 3:
                    print 'Reached!'
                    return 'reached' 
                    
                # sleep time must be under 0.5!
                rospy.sleep(0.4)
                self.elapsed_time += 0.4
                if not self.timeout == 0 and self.elapsed_time > self.timeout:
                    self.elapsed_time = 0
                    rospy.logwarn("timeout reached for navigation goal")
                    return 'preempted'
                self.arrived_client.wait_for_service()
                arrived_resp = self.arrived_client()
                
        except Exception, e:
            return 'aborted'

        return 'aborted'
    
    