#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import smach
import smach_ros

from bender_utils.ros import benpy
from bender_srvs.srv import synthesize

def getInstance(text, timeout=-1):
	
	talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
	
	try:
		talk_client(text)
		rospy.loginfo('Bender says: ' + text)
		
		if timeout == -1:
			timeout = len(text)/8
		rospy.sleep(timeout)
	except rospy.ServiceException, e:
		rospy.logerr("Service exception raised while trying to say something. " + str(e))

class State(smach.State):

    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','aborted','preempted'],input_keys=['text','timeout'])
        self.talk_client = rospy.ServiceProxy('/bender/speech/synthesizer/synthesize',synthesize)
        
    def execute(self, userdata):
		
        try:
            self.talk_client.wait_for_service()
            self.talk_client(userdata.text)
    
            if userdata.timeout < 0.0:
                # handle timeout ourselves
                pass
    
            elif userdata.timeout > 0.0:
                rospy.sleep(userdata.timeout)
            return 'succeeded'
        except rospy.ServiceException, e:
            return 'aborted'


if __name__ == '__main__':

	rospy.init_node('Talk')
	
	# timed instance
	#getInstance('manual synthesize instance for 6 seconds', 6)
	
	# time automated instance
	#getInstance('automated synthesize instance')
	

