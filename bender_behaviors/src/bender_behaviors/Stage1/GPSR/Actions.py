#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_behaviors')
import rospy
import time
import datetime
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import String as string_srv

from bender_macros.skills import Pick_table
#from bender_behaviors.Stage1.GPSR import Place
#from bender_behaviors.Stage1.GPSR import Answer
# from bender_behaviors.Stage1.GPSR import Give
from bender_behaviors.Stage1.GPSR import ParseOrder
# from bender_behaviors.Stage1.GPSR import TakeThis
from bender_macros.skills.Approach import ApproachWave, ApproachPerson
from bender_macros.skills import PersonActions     
from bender_macros.speech import WaitCommand, Talk
from bender_macros.nav import GoToPlace, ApproachToTable
from bender_macros.vision import WaitOpenDoor, FindObject
from bender_macros import GoForObject
from bender_macros.interaction import FollowMe
from bender_macros.interaction import FollowToRoom, Offer
from bender_macros.speech import AskForConfirmation, AskandConfirmation, AskQuestion
from bender_macros.head import MoveAsus

from bender_macros.arm import GiveObject
from bender_macros.arm import ReceiveObject
from bender_macros.arm import PlaceTable
from bender_core import benpy
# - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - #
#                  S t a t e    D e f i n i t i o n s                   #
# . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . - . #


            
class Actions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted','next','GO','ASKNAME','APROACH','WAVING','FIND PERSON','FIND OBJECT','PLACE','FOLLOW','COUNT OBJECT','REPORT','GRASP','TAKE THIS','BRING','ANSWER','OFFER','GIVE'],
                             input_keys=['selected_position','init_place','request_person','request_place','request_object','request_action','request_nactions'],
                             output_keys=['selected_position','object_name','person_name'])
        self.count=0

    def execute(self, userdata):
        rospy.loginfo('Executing state Actions Instructions')

        i = self.count
        self.count+=1
        if self.count == (userdata.request_nactions+1):
            self.count = 0
            return 'succeeded'

        if userdata.request_action[i] == "go":
            
            if not userdata.request_place[i] == "me":
                userdata.selected_position = userdata.request_place[i]
            else:
                userdata.selected_position = userdata.init_place
            Talk.getInstance('I will go to ' + userdata.selected_position , 3)
            return 'GO'
        if userdata.request_action[i] == "find person":
            userdata.person_name = userdata.request_person[i]
            return 'FIND PERSON'
        if userdata.request_action[i] == "waving":
            #userdata.person_name = userdata.request_person[i]
            return 'WAVING'
        if userdata.request_action[i] == "find object":
            userdata.object_name = userdata.request_object[i]
            return 'FIND OBJECT'
        if userdata.request_action[i] == "place":
            #userdata.object_name = userdata.request_object[i]
            return 'PLACE'
        if userdata.request_action[i] == "aproach":
            userdata.selected_position = userdata.request_place[i]
            return 'APROACH'
        if userdata.request_action[i] == "follow": 
        	userdata.selected_position = userdata.request_place[i]
        	return 'FOLLOW'
        if userdata.request_action[i] == "count object":    return 'COUNT OBJECT'
        if userdata.request_action[i] == "report":  return 'REPORT'
        if userdata.request_action[i] == "grasp":
            userdata.object_name = userdata.request_object[i]
            #userdata.object_name = "musculo"
            return 'GRASP'

        #person actions 
        if userdata.request_action[i] == "bring":
            Talk.getInstance('I will bring the object', 3)
            userdata.object_name = userdata.request_object[i]
            return 'BRING'
        if userdata.request_action[i] == "offer":
            userdata.selected_position = userdata.request_place[i]
            userdata.object_name = userdata.request_object[i]
            return 'OFFER'
        if userdata.request_action[i] == "take this": return 'TAKE THIS'
        if userdata.request_action[i] == "answer":  return 'ANSWER'
        if userdata.request_action[i] == "give": return 'GIVE'
        if userdata.request_action[i] == "ask name" or userdata.request_action[i] == "ask lastname" or userdata.request_action[i] == "ask nickname": 
            return 'ASKNAME'

        #Simple actions 
        if userdata.request_action[i] == "tell name team":  PersonActions.Tell_TeamName()
        if userdata.request_action[i] == "tell name":   PersonActions.Tell_Name()
        if userdata.request_action[i] == "tell time":   PersonActions.TellTime_Now()
        if userdata.request_action[i] == "tell date":   PersonActions.TellDate_Now()
        if userdata.request_action[i] == "tell day today":  PersonActions.TellDay_Today()
        if userdata.request_action[i] == "tell day tomorrow":   PersonActions.TellDay_Tomorrow()
        if userdata.request_action[i] == "tell day month":  PersonActions.TellDay_Month()
        if userdata.request_action[i] == "tell day week":   PersonActions.TellDay_Week()
        if userdata.request_action[i] == "introduce":   PersonActions.Tell_Introduce()

        return 'next'
           

class ActionNotImplemented(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             output_keys=['result_report'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ActionNotImplemented')

        Talk.getInstance('Sorry, but I dont have implemented this action', 4)

        rospy.sleep(1)
        
        return 'succeeded'
      
class CountObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             input_keys=['result_report'],
                             output_keys=['result_report'])
    
        self.plane_on_client = benpy.ServiceProxy('/planeActive', Onoff)
        self.plane = benpy.ServiceProxy('/nObjects', ID)
        
        self.nobjs = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CountObject')
        
        Talk.getInstance('I will count the objects', 3)
        
    
        try:
            self.plane_on_client(True)
            rospy.sleep(0.5)
            n = self.plane(1)
            self.plane_off_client(False)
            
            self.nobjs = n.ID 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
         

        if self.nobjs > 0:
          tim = str(self.nobjs)+' objects in the table'
        else:
          tim = 'Sorry, I not found any objects in the table'
              
        userdata.result_report += tim
        return 'succeeded'
      
      
class Report(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                   input_keys=['result_report'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Report')
    

        rp = userdata.result_report
        if rp == '':
            rp = "I dont have any to report to you"

        print rp
        Talk.getInstance(rp, 3)
       
        return 'succeeded'    


class SelecOffer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obj','drink','succeeded','aborted','preempted'],
                             io_keys=['request_person','request_place','request_object','request_action','request_nactions','object_name'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SelecOffer')

        if "drink" in userdata.object_name or "drinks" in userdata.object_name :
            return 'drink'
        if "object" in userdata.object_name or "objects" in userdata.object_name :
            return 'obj'
            
        return 'drink' 


class Add_report(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                             io_keys=['temp_report','result_report'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Add_report')

        userdata.result_report =  userdata.result_report + ", "+userdata.temp_report

        return 'succeeded'  

    

def getInstance():

    sm = smach.StateMachine(outcomes=['unsucceeded','succeeded','aborted','preempted'],
        input_keys=['init_place','request_person','request_place','request_object','request_action','request_nactions'])

    sm.userdata.map_name = 'map.sem_map'

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sm.userdata.avoid_that_place_name = 'wdys'
    sm.userdata.pre_avoid_that_place_name = 'avoid_that'
    sm.userdata.wdys_place = 'kitchen'
    sm.userdata.exit_pre  = 'exit_pre'
    sm.userdata.exit_post = 'exit_post'
    sm.userdata.request_actual= ""   
    #person if found a person, object if take the object
    sm.userdata.person_name = ""
    sm.userdata.selected_position = ""
    sm.userdata.result_report = ""
    sm.userdata.temp_report = ""
    sm.userdata.selected_arm = "/right_arm"
    sm.userdata.distance_approach_table = 0.55

    sm.userdata.textinit = 'what can I do for you?'
    sm.userdata.timeout = 3
    sm.userdata.dicc = 'gpsr'    
    
    sm.userdata.textcontinue = 'Do you have another request?'
    sm.userdata.timeoutcontinue = 2.5

    sm.userdata.textofferD = "what drink can i offer to you?"
    sm.userdata.textofferO = "what object can i offer to you?"
    sm.userdata.textaskname = "what is your name?"
    sm.userdata.diccN = 'name' 
    sm.userdata.diccD = 'drink' 
    sm.userdata.diccO = 'object' 
    sm.userdata.timeoutanswer = 10

    sm.userdata.deg_angle_count = 45

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    with sm:

        smach.StateMachine.add('ACTIONS',Actions(),
                transitions={'succeeded':'CONFIRM',
                            'GO':'GO',
                            'FIND PERSON':'FIND PERSON',
                            'FIND OBJECT':'FIND OBJECT',
                            'PLACE':'PLACE',
                            'WAVING':'WAVING',
                            'FOLLOW':'FOLLOW',
                            'COUNT OBJECT':'MOVE ASUS',
                            'REPORT':'REPORT',
                            'GRASP':'GRASP',
                            'TAKE THIS':'TAKE THIS',
                            'BRING':'BRING',
                            'ANSWER':'ANSWER',
                            'OFFER':'OFFER',
                            'APROACH':'APROACH',
                            'GIVE':'GIVE',
                            'ASKNAME':'ASKNAME',
                            'next':'ACTIONS'
                            }
        )
        smach.StateMachine.add('CONFIRM',AskForConfirmation.getInstance(),
                            transitions={'yes':'succeeded',
                                'no':'unsucceeded'},
                            remapping={'text':'textcontinue',
                                  'timeout':'timeoutcontinue'}
        )
        smach.StateMachine.add('GO',GoToPlace.getInstance(),
               transitions={'succeeded':'ACTIONS'},
               remapping={'place_name':'selected_position',
                          'map_name':'map_name'})
        smach.StateMachine.add('APROACH',ApproachToTable.getInstance(),
               transitions={'succeeded':'ACTIONS'},
               remapping={'table_name':'selected_position',
                          'map_name':'map_name',
                          'desired_distance':'distance_approach_table'})
        smach.StateMachine.add('FIND PERSON',ApproachPerson.getInstance(),
               transitions={'succeeded':'ACTIONS',
                            'notfound':'ADD_REPORT'},
                remapping = {'report':'temp_report'}
        )
        smach.StateMachine.add('FIND OBJECT',GoForObject.getInstance(),
                transitions = {'succeeded':'ACTIONS',
                           'not_grabbed':'ACTIONS',
                           'not_found':'ACTIONS'},
                remapping = {'object_name':'object_name',
                         'grasp_arm':'grasp_arm'}
        )   
        smach.StateMachine.add('GIVE',GiveObject.getInstance(),
               transitions={'succeeded':'ACTIONS'}
        )
        smach.StateMachine.add('PLACE',PlaceTable.getInstance(),
               transitions={'succeeded':'ACTIONS'}
        )
        smach.StateMachine.add('FOLLOW',FollowMe.getInstance(),
               transitions={'succeeded':'ACTIONS'},
                remapping = {'room':'selected_position'}
        )
        smach.StateMachine.add('WAVING',ApproachWave.getInstance(),
               transitions={'succeeded':'ACTIONS'}
        )
        smach.StateMachine.add('MOVE ASUS',MoveAsus.getInstance(),
               transitions={'succeeded':'COUNT OBJECT'},
               remapping={'deg_angle':'deg_angle_count'})
        
        smach.StateMachine.add('COUNT OBJECT',CountObject(),
               transitions={'succeeded':'ACTIONS'})
        smach.StateMachine.add('REPORT',Report(),
               transitions={'succeeded':'ACTIONS'})
        smach.StateMachine.add('GRASP',Pick_table.getInstance(),
               transitions={'succeeded':'ACTIONS',
                'notGrabb':'ADD_REPORT'},
                remapping={'object_name':'object_name',
                'report':'temp_report'})
        smach.StateMachine.add('TAKE THIS',ReceiveObject.getInstance(),
                transitions={'succeeded':'ACTIONS'},#TODO not grabbed colocar en report
                remapping={'selected_arm':'grasp_arm'})
        smach.StateMachine.add('BRING',GoForObject.getInstance(),
                transitions = {'succeeded':'ACTIONS',
                           'not_grabbed':'ACTIONS',
                           'not_found':'ACTIONS'},
                remapping = {'object_name':'object_name',
                         'grasp_arm':'grasp_arm'}
        )
        smach.StateMachine.add('ANSWER',AskQuestion.ask('','direct'),
            transitions = {'understand':'ACTIONS',
                            'time over':'ANSWER'},
                remapping = {'timeout':'timeoutanswer'}
        )
        smach.StateMachine.add('OFFER',SelecOffer(),
               transitions={    'obj':'AskObject',
                                 'drink':'AskDrink'}
        )
        smach.StateMachine.add('AskDrink',Offer.getInstance(),
               transitions={'succeeded':'ACTIONS'},
                remapping = {'textoffer':'textofferD',
                            'dicc':'diccD'}
                            )
        smach.StateMachine.add('AskObject',Offer.getInstance(),
               transitions={'succeeded':'ACTIONS'},
                remapping = {'textoffer':'textofferO',
                            'dicc':'diccO'}
                            )
        smach.StateMachine.add('ASKNAME',AskandConfirmation.getInstance(),
               transitions={'succeeded':'ACTIONS'},
               remapping={'text':'textaskname',
                          'timeout':'timeout',
                          'dic_ask':'diccN'})
        smach.StateMachine.add('ADD_REPORT',Add_report(),
               transitions={'succeeded':'ACTIONS'}) 
    return sm

# main
if __name__ == '__main__':

    rospy.init_node('ACTIONS')

    sm = getInstance()

    # introspection server
    sis = smach_ros.IntrospectionServer('actions', sm, '/ACTIONS_SM')
    sis.start()
    outcome = sm.execute()
    sis.stop()
