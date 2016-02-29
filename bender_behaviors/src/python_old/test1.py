#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros

from bender_msgs.msg import *
from bender_srvs.srv import *

matches = [-1,-1,-1]
distances= [0,0,0]

# define state Foo
class FindFace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Face Ready','Find Face'])
        self.counter = 0
	

    def execute(self, userdata):
#        rospy.loginfo('Executing state FOO')
        face_recognition = rospy.ServiceProxy('/bender_vision/face_recognition/recognize', FaceRecognition)
        try:
		resp=face_recognition(0,"")
	except rospy.ServiceException, e:
		self.counter=0
		return 'Find Face'
	matches[self.counter]=resp.face_index
	distances[self.counter]=resp.distance
	self.counter+=1
	if self.counter==3:
	    self.counter=0
            return 'Face Ready'
        else:
            return 'Find Face'


# define state Bar
class DetectFace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Find Face'])

    def execute(self, userdata):
	
	print "Recognition Results:"
	print "Best matches: "
	print matches
	print "Distance: " 
	print distances
#	sys.stdin.read(1)
#        rospy.loginfo('Executing state BAR')
        return 'Find Face'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')
    #rospy.wait_for_server('/camera_service')
    print "waiting for camera_service"
    #camera_service = rospy.ServiceProxy('camera_service', ImageService)
    face_DB = rospy.ServiceProxy('/bender_vision/face_recognition/read_im_db', FaceRecognition)
    resp=face_DB(0,"description.txt")
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FindFace', FindFace(), 
                               transitions={'Find Face':'FindFace', 
                                            'Face Ready':'DetectFace'})
        smach.StateMachine.add('DetectFace', DetectFace(), 
                               transitions={'Find Face':'FindFace'})
	
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
