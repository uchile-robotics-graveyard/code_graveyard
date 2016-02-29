#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
import re
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
import yaml, genpy
import rospy, rospkg

from bender_macros.speech import Talk

def load_yaml(name):
    rospy.loginfo('Executing state FindActions')
    rospack = rospkg.RosPack()
    parser_path = rospack.get_path('bender_utils')+'/config/mapper/gspr.yaml'

    try:
        with open(parser_path, 'r') as f:
            # load all documents
            parser_data = yaml.load(f)
            if parser_data is None:
                raise yaml.YAMLError("Empty files not allowed")
    except yaml.YAMLError as e:
        rospy.logerr('Invalid YAML file: %s' % (str(e)))

    return parser_data[name]

class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'],
            #io_keys=['recognized_phrase'],
            io_keys=['temp_phrase','temp_action','request_nactions','request_person','request_place','request_object','request_action'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Setup')
        userdata.request_nactions = 0
        userdata.request_person = []
        userdata.request_place = []
        userdata.request_object = []
        userdata.request_action = []

        userdata.temp_phrase = []
        #userdata.recognized_phrase = "take the crackers from the shelf and take them to Susan in the kitchen"
        rospy.sleep(0.1)
        return 'succeeded'

class SimplifyOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['succeeded','aborted','preempted'],
            input_keys=['recognized_phrase'],
             output_keys=['recognized_phrase'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SimplifyOrder')
        
        REMOVE_LIST = load_yaml('eliminate')#rospy.get_param('/ParseOrder/eliminate')
        
        sentence = userdata.recognized_phrase.split(" ")
        newsente = ' '.join([i for i in sentence if i not in REMOVE_LIST])
        
        rospy.loginfo('New Phrase : '+ newsente)
        userdata.recognized_phrase = newsente

        return 'succeeded'


class FindActions(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['succeeded','aborted','preempted'],
            input_keys=['recognized_phrase'],
            output_keys=['temp_phrase','temp_action'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing state FindActions')
        ACTION_LIST  = load_yaml('actions')
        #ACTION_LIST  = rospy.get_param('/ParseOrder/actions')
        
        pattern = '|'.join(map(re.escape,ACTION_LIST))
        
        sentence = re.split(pattern,userdata.recognized_phrase)
        while "" in sentence : sentence.remove("")
            
        actions = re.findall(pattern, userdata.recognized_phrase)
        
        print pattern
        userdata.temp_phrase = sentence
        userdata.temp_action = actions
        print sentence
        print actions
        #userdata.request_nactions = len(actions)
        if len(sentence) != len(actions) :
            rospy.loginfo('The sentence is incomplete')
            return 'aborted'


        return 'succeeded'


class EvaluateActions(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['succeeded','aborted','preempted'],
            io_keys=['temp_phrase','temp_action'])
        self.DRINKS_LIST =  load_yaml('drinks')#rospy.get_param('/ParseOrder/drinks')
        self.FOOD_LIST =  load_yaml('food')#rospy.get_param('/ParseOrder/food')
        self.LOCATION_LIST = load_yaml('location')#rospy.get_param('/ParseOrder/location')
        self.MANIPULATE_LOCATION_LIST = load_yaml('manipulate_location')#rospy.get_param('/ParseOrder/manipulate_location')
        
        self.pattern_drinks = '|'.join(map(re.escape,self.DRINKS_LIST))
        self.pattern_food = '|'.join(map(re.escape,self.FOOD_LIST))
        self.pattern_location = '|'.join(map(re.escape,self.LOCATION_LIST))
        self.pattern_manipulation_location = '|'.join(map(re.escape,self.MANIPULATE_LOCATION_LIST))
        
    def execute(self,userdata):
        rospy.loginfo('Executing state EvaluateActions')

        objD = re.findall(self.pattern_drinks, userdata.temp_phrase[0])
        objF = re.findall(self.pattern_food,  userdata.temp_phrase[0])
        
### Para distinguir un 'take' interpretado como 'bring' de un 'take' interpretado como 'grasp'
### nos fijamos en la cantidad de lugares mencionados en la frase.  Un 'bring' no necesita de un lugar
### mientras que un 'grasp' si lo necesita.  

        locA = []
        locB = []

        for i  in range(len(userdata.temp_phrase)) : locA = locA + re.findall(self.pattern_location, userdata.temp_phrase[i])
        for i  in range(len(userdata.temp_phrase)) : locB = locB + re.findall(self.pattern_manipulation_location, userdata.temp_phrase[i])

        print  userdata.temp_action[0]
        print len(objD) + len(objF) 

####  Aqui imponemos la condicion de los lugares.
        if "take" == userdata.temp_action[0] and len(objD) + len(objF) > 0 and len(locA) + len(locB) < 2:
            print "asd"
            userdata.temp_action[0] = "bring"
        
        print userdata.temp_action
        return 'succeeded'

class SeparateActions(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['identifyphrase','succeeded','aborted','preempted'],
            input_keys=['temp_phrase','temp_action','id_phrase','request_action','request_person','request_place','request_object','request_nactions'],
            output_keys=['evaluate_phrase','evaluate_action','id_phrase','request_nactions'])
        self.count = 0
        
    def execute(self,userdata):
        rospy.loginfo('Executing state SeparateActions')
        if self.count >= len(userdata.temp_action): 
            userdata.request_nactions = len(userdata.request_action)
            print userdata.request_nactions
            print userdata.request_action
            print userdata.request_person
            print userdata.request_place
            print userdata.request_object
            self.count = 0
            return 'succeeded'

        if len(userdata.temp_phrase) > self.count and len(userdata.temp_action) > self.count :
                userdata.evaluate_phrase = userdata.temp_phrase[self.count]
                userdata.evaluate_action = userdata.temp_action[self.count]
                userdata.id_phrase = self.count
        
                self.count=userdata.id_phrase +1  
                return 'identifyphrase'

        self.count=userdata.id_phrase +1  
        print str(self.count)+" - "+str(len(userdata.temp_action))
        print userdata.temp_phrase
        return 'preempted'


class IdentifyActions(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes = ['succeeded','aborted','preempted'],
                input_keys=['require_information','evaluate_phrase','evaluate_action','id_phrase','request_nactions','request_action','request_person','request_place','request_object'],
                output_keys=['require_information','request_action','request_person','request_place','request_object','id_phrase'])

            self.PERSON_LIST = load_yaml('names')#rospy.get_param('/ParseOrder/names')
            self.LOCATION_LIST =  load_yaml('location')#rospy.get_param('/ParseOrder/location')
            self.MANIPULATE_LOCATION_LIST =  load_yaml('manipulate_location')#rospy.get_param('/ParseOrder/manipulate_location')
            self.DRINKS_LIST =  load_yaml('drinks')#rospy.get_param('/ParseOrder/drinks')
            self.FOOD_LIST =  load_yaml('food')#rospy.get_param('/ParseOrder/food')

            self.pattern_person = '|'.join(map(re.escape,self.PERSON_LIST))
            self.pattern_location = '|'.join(map(re.escape,self.LOCATION_LIST))
            self.pattern_manipulation_location = '|'.join(map(re.escape,self.MANIPULATE_LOCATION_LIST))
            self.pattern_drinks = '|'.join(map(re.escape,self.DRINKS_LIST))
            self.pattern_food = '|'.join(map(re.escape,self.FOOD_LIST))

            self.use_person = 0
            self.use_loc = 0
            self.use_obj = 0

            self.actual_position = ""

        def execute(self,userdata):
            rospy.loginfo('Executing state IdentifyActions')
            rospy.loginfo(userdata.evaluate_action + " "+ userdata.evaluate_phrase)

            phrase = userdata.evaluate_phrase
            acttmp = userdata.evaluate_action.strip()

            self.use_person = 0
            self.use_loc = 0
            self.use_obj = 0
            if acttmp == "bring":
                if phrase == " it to" or phrase == " them to": 
                    phrase =  "it to me"

            #Acciones que necesitan confirmacion
            #TODO utilizar require_information, verificar si la informacion viene en la frase 
            #y sino se debe preguntar
            if acttmp == "bring":
                    [found, obj] = self.evaluate_object(userdata,phrase)
                    print found
                    if found:
                            print "bring : found true"
                            self.use_obj = 1
                            self.addop(userdata,acttmp,"","",obj) 
                    else:
                            print "bring : found false"
                            categ = self.evaluate_category(userdata,phrase)
                            if not categ =="":
                                self.use_obj = 0

        
            #Navegacion
            if ("what" not in phrase and "at " in phrase) or "in " in phrase or "from " in phrase or "which is in" in phrase:
                    if "at " in phrase : place = phrase.split("at ")
                    if "in " in phrase : place = phrase.split("in ")
                    if "from " in phrase : place = phrase.split("from ")
                    loc = place[1].strip()
                    self.evaluate_location(userdata,loc)
                    phrase = place[0]
            

            #### Para evitar errores con frases del tipo '...follow her TO THE <place>' se impone la condicion 'follow' not in
            else:
                    if "guide" not in acttmp and "follow" not in acttmp:#  and "bring" not in acttmp :
                            print "nave : found false"
                            self.evaluate_location(userdata,phrase)
            print phrase            
        
            #Person   
            if acttmp == "find"  or acttmp == "waving" or acttmp == "calling" or acttmp == "follow":
                    [found, person] = self.evaluate_person(userdata,phrase)
                    [found1, obj1] = self.evaluate_object(userdata,phrase)
                 # temp = acttmp
                    if acttmp == "find": 
                            if found:
                                    acttmp +=" person"   
                                    self.addop(userdata,acttmp,person,"","")       
                            if found1:
                                    self.addop(userdata,"find object","",self.actual_position, obj1) 
                    if acttmp == "waving" or acttmp == "calling": 
                            if found:
                                    self.addop(userdata,"waving",person,"","") 
                    if acttmp == "follow": 
                            if found:
                                    self.addop(userdata,acttmp,person,"","") 
            else:  
                    [found, person] = self.evaluate_person(userdata,phrase)
                    if found and ("her" not in person and "him" not in person):
                            self.addop(userdata,"find person",person,"","")  
                            self.use_person = 1
            if acttmp == "look":
                    [found1, obj1] = self.evaluate_object(userdata,phrase)
                    if found1:
                            self.addop(userdata,"find object","",self.actual_position, obj1)                   
            if acttmp == "bring":
                    self.addop(userdata,"give","","","") 

            if acttmp == "tell" or acttmp == "say" or acttmp == "speak":
                    print "verb speak"
                    print phrase
                    if "name" in phrase and "team" in phrase:
                            self.addop(userdata,"tell name team","","","") 
                    if "name" in phrase and "team" not in phrase:
                            self.addop(userdata,"tell name","","","") 
                    if "time" in phrase:
                            print "time"
                            self.addop(userdata,"tell time","","","") 
                    if "date" in phrase:
                            self.addop(userdata,"tell date","","","")     
                    if "day" in phrase and "today" in phrase:
                            self.addop(userdata,"tell day today","","","")  
                    if "day" in phrase and "tomorrow" in phrase:
                            self.addop(userdata,"tell day tomorrow","","","")  
                    if "day" in phrase and "month" in phrase:
                            self.addop(userdata,"tell day month","","","")  
                    if "day" in phrase and "week" in phrase:
                            self.addop(userdata,"tell day week","","","")  
            if acttmp == "count":
                    category = self.evaluate_category(userdata,phrase)
                    self.addop(userdata,"count object","","",category)
            if acttmp == "offer":
                    category = self.evaluate_category(userdata,phrase)
                    self.addop(userdata,acttmp,"","",category)  
            if acttmp == "report":
                    [found, person] = self.evaluate_person(userdata,phrase)
                    self.addop(userdata,"report",person,"","")  
            if "introduce"  in acttmp or "answer" in acttmp:
                    self.addop(userdata,acttmp,"","","")    
            if "put"  in acttmp or "deliver"  in acttmp:
                    self.addop(userdata,"place","","","")  
            if "leave"  in acttmp:
                    self.addop(userdata,"go","","exit","")  
            if "guide" in acttmp:
                self.addop(userdata,acttmp,"","","")  
                self.evaluate_location(userdata,phrase)
            if acttmp == "grasp" or acttmp == "get":
                    [found, obj] = self.evaluate_object(userdata,phrase)
                    if found:
                            self.addop(userdata,"grasp","","",obj) 
            if acttmp == "take":
                    if "this" in phrase:
                            self.addop(userdata,"take this","","","")   
                    elif "them" in phrase or "it" in phrase:
                            if self.use_person == 1:
                                    self.addop(userdata,"give","","","") 
                            else :
                                    self.addop(userdata,"place","","","") 
                    else:
                            [found, obj] = self.evaluate_object(userdata,phrase)
                            if found:
                                    self.addop(userdata,"grasp","","",obj) 

            if acttmp == "give":
                    if "this" in phrase:
                            self.addop(userdata,"take this","","","")   
            if acttmp == "ask":
                    if "name" in phrase:    self.addop(userdata,"ask name","","","") 
                    if "lastname" in phrase:    self.addop(userdata,"ask lastname","","","") 
                    if "nickname" in phrase:    self.addop(userdata,"ask nickname","","","") 

            return 'succeeded'
        
        def addop(self,userdata, action,person,place,obj):
                strlog = str(userdata.id_phrase)+" : action :"+action + ", person: "+person +", place: "+place+", obj:"+obj
                rospy.loginfo(strlog)
                userdata.request_action.append(action)
                userdata.request_person.append(person)
                userdata.request_place.append(place)
                userdata.request_object.append(obj)
                userdata.id_phrase +=1
        
        def evaluate_location(self,userdata,phrase,add = 1):
                phrase = phrase.strip()
                nav = re.findall(self.pattern_location, phrase)
                nav2 = re.findall(self.pattern_manipulation_location, phrase)
                ret = False
                if len(nav) > 0:
                        ret = True
                        if add==0:
                            return ret
                        place = nav[0].strip()
                        self.use_loc = 1
                        if place in " me ":
                                self.use_person=1
                        self.actual_position = place
                        self.addop(userdata,"go","",place,"") 
                else:
                        if len(nav2) > 0:
                            ret = True
                            if add==0:
                                return ret
                            self.use_loc = 1
                            place = nav2[0].strip()
                            self.actual_position = place
                            self.addop(userdata,"aproach","",place,"")
                return ret

        def evaluate_person(self,userdata,phrase):
                per = re.findall(self.pattern_person, phrase)
                person = "person"
                found = False
                if len(per) > 0:
                        found = True
                        person = per[0].strip()
                return [found, person]

        def evaluate_category(self,userdata,phrase):
                category = ""
                if "drink" in phrase or "drinks" in phrase:
                        category = "drinks"
                if "food" in phrase:
                        category = "food"
                if "object" in phrase:
                        category = "object"
                return category

        def evaluate_object(self,userdata,phrase):
                obj = ""
                found = False
                objD = re.findall(self.pattern_drinks, phrase)
                objF = re.findall(self.pattern_food, phrase)
                if len(objD) > 0  :
                        obj = objD[0].strip()
                        found = True
                if len(objF) > 0  :
                        obj = objF[0].strip()
                        found = True
                return [found, obj]         
                    
                
def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                        input_keys=['recognized_phrase'],
            output_keys=['request_nactions','request_action','request_person','request_place','request_object'])

    sm.userdata.text = ''
    sm.userdata.id_phrase =0
    sm.userdata.request_nactions = 0
    sm.userdata.request_person = []
    sm.userdata.request_place = []
    sm.userdata.request_object = []
    sm.userdata.request_action = []

    sm.userdata.temp_phrase = []
    #sm.userdata.recognized_phrase = ""
    with sm:

        smach.StateMachine.add('SETUP',Setup(),
                             transitions={'succeeded':'SIMPLIFY_ORDER'}
                             )
        smach.StateMachine.add('SIMPLIFY_ORDER',SimplifyOrder(),
                             transitions={'succeeded':'FIND_ACTIONS'}
                             )
        smach.StateMachine.add('FIND_ACTIONS',FindActions(),
                             transitions={'succeeded':'EVALUATE_ACTIONS',
                                          'aborted':'aborted'}
                             )
        smach.StateMachine.add('EVALUATE_ACTIONS',EvaluateActions(),
                             transitions={'succeeded':'SEPARATE_ACTIONS'}
                             )
        smach.StateMachine.add('SEPARATE_ACTIONS',SeparateActions(),
                             transitions={'succeeded':'succeeded',
                                'identifyphrase':'IDENTIFY_ACTIONS',
                                'preempted':'SEPARATE_ACTIONS'}
                             )
        smach.StateMachine.add('IDENTIFY_ACTIONS',IdentifyActions(),
                             transitions={'succeeded':'SEPARATE_ACTIONS'},
                             )
    return sm


if __name__ == '__main__':

    rospy.init_node('ParseOrder')

    sm = getInstance()

    test16 = 'bring me a crackers'
    test15 = 'bring me some drinks from a dinner-table'
    test14 = 'move to the kitchen, move to the kitchen, and move to the kitchen'
    test13 = 'move to the kitchen, move to the kitchen and leave the apartment'
    test12 = 'move to the kitchen, grasp the crackers, and bring it to me'
    test11 = 'move to the kitchen, grasp the crackers, and bring it to the kitchen'
    test10 = 'move to the kitchen, grasp the crackers, and put it in the trash bin'
    test9 = 'move to the kitchen, find a person, and introduce yourself'
    test8 = 'move to the kitchen, find a person, and guide it to the exit'
    test7 =  'go to the dinner-table find a person and tell the time'
    test6 =  'go to the kitchen find a person and follow her'
    test5b =  'go to the dinner-table grasp the crackers and take them to susan in the living'
    test5a =  'go to the dinner-table grasp the crackers and take them to the person in the living'
    test5 =  'go to the dinner-table grasp the crackers and take them to the tv'
    test4a =  'go to the shelf count the drinks and report to the person in the living'
    test4 =  'go to the shelf count the drinks and report to me'
    test3 =  'take this object and bring it to susan at the hall'
    test2 =  'bring a coke to the person in the living-room and answer him a question'
    test1 =  'offer a drink to the person at the door'

    test5b =  'go to the dinner-table grasp the crackers and bring them to'

    ud = smach.UserData()
    ud.recognized_phrase = test5
    # introspection server
    sis = smach_ros.IntrospectionServer('ParseOrder', sm, '/PARSE_ORDER_SM ')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
