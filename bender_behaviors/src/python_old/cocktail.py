#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import math
import time

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from bender_msgs.msg import *
from bender_srvs.srv import *

pub = rospy.Publisher('/cmd_vel', Twist)

Pt = -0.0007 # Tralacion
Pr = -0.03   # Rotacion
lin_max = 0.7
thetamax = 1.0

DIST_OBJ = 1000

follow_ID=-1

# define state Foo
class GoToParty(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'])

    def execute(self, userdata):
        return 'Arrived'
        
    def callback(self, data):
        return
    
class SearchWaveing(smach.State):
    def __init__(self, n_persons):
        smach.State.__init__(self, outcomes=['Detected','Not Detected'],input_keys=[''],output_keys=['waveing_detection'])

    def execute(self, userdata):
        return 'Detected'
        
    def callback(self, data):
        return

class GoToWaveing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'],input_keys=['waveing_position'])

    def execute(self, userdata):
        return 'Arrived'
        
    def callback(self, data):
        return

class GoToWaveing(smach.State):
    def __init__(self, n_persons):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'])

    def execute(self, userdata):
        return 'Arrived'
        
    def callback(self, data):
        return

class Chat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Ready','Not Ready'])

    def execute(self, userdata):
        return 'Ready'
        
    def callback(self, data):
        return

class GoToKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'])

    def execute(self, userdata):
        return 'Arrived'
        
    def callback(self, data):
        return
# define state Foo
class InitFollow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Is Init','Is Not Init'])
        self.counter = 0
        self.is_init=False
        rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,self.callback)

    def execute(self, userdata):
        #time.sleep(5)
#        rospy.loginfo('Executing state FOO')
        if self.is_init:
            return 'Is Init'
        else:
            return 'Is Not Init'
        
        
    def callback(self, data):
        global follow_ID
        if self.is_init:
            return
        for i in range(len(data.ID)):
            if abs(data.Distance[i]-DIST_OBJ) < 300 and abs(data.Theta[i]) < 5 :
                follow_ID=data.ID[i]
                #print follow_ID
                self.is_init=True
 
# define state Bar
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Following'])
        self.counter=0
        self.twist = Twist() # inicializa en ceros
        self.currR=0
        self.currTheta=0
        self.lost=True
        rospy.Subscriber('/KinectTracker_Estimation', KinectTrackerData,self.callback2)
  
    def execute(self, userdata):
        #time.sleep(10)
        if self.lost:
            self.twist.linear.x=0
            self.twist.angular.z=0
            pub.publish(self.twist) 
            print("Kinect Tracking Lost")
            return 'Following'
        print("Current Radius: ",self.currR)
        #print("Face Offset: ",(resp.BBoxes[main_id].x + (resp.BBoxes[main_id].width/2.0)-CAM_WIDTH/2)/resp.BBoxes[main_id].width)
        print("Current Theta: ",self.currTheta)

        self.twist.linear.x = Pt*(DIST_OBJ-self.currR)
        self.twist.linear.x = min(self.twist.linear.x,lin_max)
        self.twist.linear.x = max(self.twist.linear.x,-lin_max)
        
#        self.twist.angular.z = -Pr*((resp.BBoxes[main_id].x + (resp.BBoxes[main_id].width/2.0)-CAM_WIDTH/2)/resp.BBoxes[main_id].width)
        self.twist.angular.z =-Pr*self.currTheta
        self.twist.angular.z=min(self.twist.angular.z,thetamax)
        self.twist.angular.z=max(self.twist.angular.z,-thetamax)
        pub.publish(self.twist) 
        
        
#       sys.stdin.read(1)
#       rospy.loginfo('Executing state BAR')
        return 'Following'
        
    def callback2(self, data):
        global follow_ID
        index=-1
        #print("Ubicadoooooo")
        for i in range(len(data.ID)):
            if data.ID[i] == follow_ID:
                index=i
                break
        print follow_ID
        print index
        if index == -1:
            self.lost=True
            return
        else:
            self.lost=False
        self.currR=data.Distance[index]
        self.currTheta=data.Theta[index]


def main():
    rospy.init_node('cocktail_state_machine')    

    #rospy.wait_for_server('/camera_service')
    #rospy.wait_for_service('/goalServer/sendGoal') esperar por:
    #print "waiting for camera_service"
    #camera_service = rospy.ServiceProxy('camera_service', ImageService)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoToParty', GoToPlace("living room"), 
                               transitions={'Arrived':'SearchWaveing', 
                                            'Not Arrived':'GoToParty'})
        smach.StateMachine.add('SearchWaveing', SearchWaveing(3), 
                               transitions={'Detected':'GoToWave',
                                            'Not Detected':'SearchWaveing'})
        smach.StateMachine.add('GoToWaveing', GoToWaveing(), 
                               transitions={'Arrived':'Chat',
                                            'Not Arrived':'GoToWaveing'})
        smach.StateMachine.add('Chat', Chat, 
                               transitions={'Ready':'GoToKitchen',
                                            'Not Ready':'GoToParty'})
        smach.StateMachine.add('GoToKitchen', GoToPlace("kitchen"), 
                               transitions={'Arrived':'GoToParty2', 
                                            'Not Arrived':'GoToKitchen'})
        smach.StateMachine.add('GoToParty2', GoToPlace("living room"), 
                               transitions={'Arrived':'outcome4',#'GotoPoses', 
                                            'Not Arrived':'GoToParty2'})
        #smach.StateMachine.add('GoToPoses', GoToPlace(3), 
        #                       transitions={'Arrived':'FindUser', 
        #                                    'Not Arrived':'GoToParty2'})
        #smach.StateMachine.add('FindUser', FindUser(), 
        #                       transitions={'UserFound':'Deliver', 
        #                                    'User Not Found':'Search User'})
        
        
        
        
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
