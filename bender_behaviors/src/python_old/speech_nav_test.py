#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import smach
import smach_ros
import math

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import *
from rospy.timer import sleep
from bender_msgs.msg import *
from bender_srvs.srv import *


recognized_order = ''

class order_recognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Order Not Recognized','Order Recognized'])
        rospy.Subscriber('speech_recognizer/output', String,self.recognition_callback)
        
        self.recognized = False
        self.running = False
        
    def execute(self, userdata):
        
        self.running = True
        if self.recognized:
            self.running = False
            self.recognized = False
            return 'Order Recognized'
        else:
            return 'Order Not Recognized'
        
    def recognition_callback(self, recognition):
        global recognized_order
        
        if self.running:             
            if recognition.data == 'bender go to the kitchen':
                recognized_order = 'kitchen'
                self.recognized = True
            elif recognition.data == 'bender go to the diningroom':
                recognized_order = 'diningroom'
                self.recognized = True
            elif recognition.data == 'bender go to the livingroom':
                recognized_order = 'livingroom'
                self.recognized = True
            elif recognition.data == 'bender go to the bedroom':
                recognized_order = 'bedroom'
                self.recognized = True
            elif recognition.data == 'bender go to the hall':
                recognized_order = 'hall'
                self.recognized.data = True
            elif recognition.data == 'bender go to the corridor':
                recognized_order = 'corridor'
                self.recognized = True
            print recognized_order      
        
        
class order_actuation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Working','Order Done','End'])
        self.talk = rospy.ServiceProxy('speech_synthesizer/synthesize', synthesize)
        rospy.Subscriber('/goal_reached',Bool,self.goal_reached_callback)
        self.working = False
        self.done_working = False
        self.semCaller = rospy.ServiceProxy('/semantic_map_server/get', sem_map_get)
        self.setGoal=rospy.ServiceProxy('/goalServer/sendGoal', sendGoal)
    
    def goToPlace(self,place):
        self.talk("I am going to the "+place)
        resp=self.semCaller(place)
        print resp
        self.setGoal(resp.semObj.pose,0)
        
    def execute(self, userdata):
        global recognized_order
        
        if self.done_working:
            self.done_working = False
            self.working = False
            return 'Order Done'
        else:
            if self.working == False:
                if recognized_order == 'kitchen':
                    self.goToPlace('kitchen')
                    return 'End'
                elif recognized_order == 'diningroom':
                    self.goToPlace('dining room')
                    return 'End'
                elif recognized_order == 'livingroom':
                    self.goToPlace('living room')
                    return 'End'
                elif recognized_order == 'bedroom':
                    self.talk("I am going to the "+recognized_order)
                elif recognized_order == 'hall':
                    self.goToPlace('hall')
                    return 'End'
                elif recognized_order == 'corridor':
                    self.goToPlace('corridor')
                    return 'End'
                else:
                    self.talk("what the fuck, something is wrong")
                
                self.working = True
            return 'Working'
    
    def goal_reached_callback(self,done):
        if self.working:
            if done.data == True:
                self.done_working = True
    
def main():
    rospy.init_node('speech_nav_state_machine') 
    load_dict = rospy.ServiceProxy('speech_recognizer/load_dictionary', load_dictionary_service)   
    load_dict("move")
    
    start = rospy.ServiceProxy('speech_recognizer/start', Empty)
    rospy.wait_for_service('speech_recognizer/start')
    start()  
    mapLoader = rospy.ServiceProxy('/semantic_map_server/load', save_load_Map)
    rospy.wait_for_service('/semantic_map_server/load')
    mapLoader("map_base.sem_map")
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('order_recognition', order_recognition(), 
                               transitions={'Order Not Recognized':'order_recognition', 
                                            'Order Recognized':'order_actuation'})
        smach.StateMachine.add('order_actuation', order_actuation(), 
                               transitions={'Working':'order_actuation',
                                            'Order Done':'order_recognition',
                                            'End':'order_recognition'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
