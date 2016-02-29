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

DIST_OBJ = 1500

follow_ID=-1

PartyRoom = 'living room'
DrinkRoom = 'kitchen'
Exit = 'exit'

class CheckingDoor(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=['Closed','Open'])

    def execute(self, userdata):
        return 'Open'

class EnterArena(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'])

    def execute(self, userdata):
        return 'Arrived'

class GoToPlace(smach.State):
    def __init__(self, place):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'])

    def execute(self, userdata):
        return 'Arrived'

class SearchCallingPerson(smach.State):
    def __init__(self, n_persons):
        smach.State.__init__(self, outcomes=['Detected','Not Detected','Time Out','Time Out and Queue Empty'],input_keys=[''],output_keys=['waving_detection'])

    def execute(self, userdata):
        return 'Detected'
        
    def callback(self, data):
        return

class GoToWaving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Arrived','Not Arrived'],input_keys=['waving_position'])

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


def main():
    global PartyRoom, DrinkRoom
    rospy.init_node('cocktail')    

    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CheckingDoor', CheckingDoor(), 
                               transitions={'Closed':'CheckingDoor', 
                                            'Open':'EnterArena'})
        smach.StateMachine.add('EnterArena', EnterArena(), 
                               transitions={'Not Arrived':'EnterArena',
                                            'Arrived':'GoToParty'})
        
        smach.StateMachine.add('GoToParty', GoToPlace(PartyRoom), 
                               transitions={'Arrived':'SearchCallingPerson', 
                                            'Not Arrived':'GoToParty'})
        smach.StateMachine.add('SearchCallingPerson', SearchCallingPerson(), 
                               transitions={'Detected':'GoToClient',
                                            'Not Detected':'SearchCallingPerson',
                                            'Time Out':'GoToDrinks',
                                            'Time Out and Queue Empty':'GoToExit'})
        smach.StateMachine.add('GoToClient', GoToClient(), 
                               transitions={'Arrived':'TakeOrder',
                                            'Not Arrived':'GoToClient'})
        smach.StateMachine.add('TakeOrder', TakeOrder(), 
                               transitions={'Take Other Order':'GoToParty',
                                            'Order Queue Full':'GoToDrinks',
                                            'Not Ready':'TakeOrder'})
        
        smach.StateMachine.add('GoToDrinks', GoToPlace(DrinkRoom), 
                               transitions={'Arrived':'SearchDrinks', 
                                            'Not Arrived':'GoToDrinks'})
        smach.StateMachine.add('SearchDrinks', searchDrinks(), 
                               transitions={'Found':'GraspDrink', 
                                            'Not Found':'SearchDrinks',
                                            'No Drink In Queue':'GoToDeliver',
                                            'Time Out':'GoToExit'})
        smach.StateMachine.add('GraspDrinks', GraspDrinks(), 
                               transitions={'Grasped':'SearchDrinks', 
                                            'Not Grasped':'GraspDrinks',
                                            'Time Out':'askForDrink'})
        smach.StateMachine.add('askForDrink', askForDrink(), 
                               transitions={'Drink In Possition':'SearchDrinks', 
                                            'Asking For Drink':'askForDrink'})
        
        smach.StateMachine.add('GoToDeliver', GoToPlace(PartyRoom), 
                               transitions={'Arrived':'outcome4',#'GotoPoses', 
                                            'Not Arrived':'GoToParty2'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
