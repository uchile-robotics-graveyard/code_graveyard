#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_srvs.srv import DoorDetector
from bender_srvs.srv import DoorDetectorRequest
from bender_srvs.srv import DoorDetectorResponse



class WaitOpenDoor(smach.State):
    
    def __init__(self):
        
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        
        self.max_retry_times = 3
        self.distances = [1.0, 1.1, 1.2]
        self.current_retries = 0
        
        
    def execute(self, userdata):
        
        door_client = rospy.ServiceProxy('/bender/vision/door_open_detector/is_open', DoorDetector)
        rospy.loginfo("waiting for service: '/bender/vision/door_open_detector/is_open . . .'")
        door_client.wait_for_service()

        door_req = DoorDetectorRequest()
        
        self.current_retries = 0
        for distance in self.distances:
            
            rospy.loginfo('Checking distance: ' + str(distance))
            door_req.distance = distance
            
            while True:
            
                try:
                    door_client.wait_for_service()
                    door_resp = door_client(door_req)
                    
                except Exception, e:
                    return 'aborted'
                
                if door_resp.door_opened == True:
                    
                    rospy.loginfo('distance ok')
                    self.current_retries +=  1
                    
                    if self.current_retries >= self.max_retry_times:
                        break
                        
                rospy.sleep(0.3)
                        
            self.current_retries = 0

        return 'succeeded'

def getInstance():
        
    sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])
    
    with sm:
        
        smach.StateMachine.add('WAIT_SIGNAL',WaitOpenDoor(),
                transitions = {'succeeded':'succeeded'}
        )        

    return sm
 
# main
if __name__ == '__main__':

    rospy.init_node('wait_open_door')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('wait_open_door', sm, '/WAIT_OPEN_DOOR_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
 
    