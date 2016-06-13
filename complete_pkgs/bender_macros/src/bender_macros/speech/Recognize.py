#!/usr/bin/env python

import roslib; roslib.load_manifest('bender_macros')
import rospy
import time
import smach
import smach_ros
from std_srvs.srv import *
from bender_srvs.srv import *
from bender_msgs.msg import *
from bender_srvs.srv import String as Dict
from bender_utils.ros import benpy
import re # no borrar
from bender_macros.speech import Talk
from std_msgs.msg import String


class Recognize(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = ['succeeded','aborted','preempted'],
            input_keys = ['dictionary'],
            output_keys = ['recognized_word']
        )

        self.startSrv    = rospy.ServiceProxy('/bender/speech/recognizer/start', Empty)
        self.stopSrv     = rospy.ServiceProxy('/bender/speech/recognizer/stop', Empty)
        self.loadDictSrv = rospy.ServiceProxy('/bender/speech/recognizer/load_dictionary', load_dictionary_service)
        #self.getDictSrv = rospy.ServiceProxy('/bender/speech/recognizer/get_dictionary', Dict)

        # Subscriptions
        rospy.Subscriber('/bender/speech/recognizer/output', String, self.recognitionCB)
        # rospy.Subscriber('/bender/speech/recognizer/partial_output', std_msgs.msg.String, self.recognitionP)
        
        self.available_dictionaries = ""
        self.timeinit = 0
        self.timeslast = 0
        self.timeslastP = 0
        self.word = ""
        self.wordP = ""

        self.current_dict = ''

    def load_dict(self, dictionary):

        req  = load_dictionary_serviceRequest()
        req.dictionary = dictionary

        try:
            self.loadDictSrv(req)
            rospy.loginfo('Dictionary Loaded: ' + dictionary)
        except:
            rospy.logerr("Can't load dictionary " + dictionary)
            return 'aborted'


        
    def execute(self, userdata):

        rospy.loginfo('Executing state Recognize with Dictionary: ' + userdata.dictionary)
        #self.current_dict = self.getDictSrv('')         
        #if self.current_dict != userdata.dictionary:
        #    self.load_dict(userdata.dictionary)

        # Start recognition
        started = self.startSrv()

        self.timeinit = time.time()
        self.timeslast = time.time()
        
        self.word = ""
        # self.wordP = ""

        while self.word == "" and not rospy.is_shutdown():
            tf= time.time()
            if  tf - self.timeslast > 10:
                stop = self.stopSrv()
                Talk.getInstance('I am sorry, can you repeate please',3.5)
                return 'preempted'
            # if  tf - self.timeslastP > 2 and tf - self.timeslast >  5 and len(self.word) == 0 and len(self.wordP)>2:
            #     self.word = self.wordP

        # Stop recognition
        stop = self.stopSrv()
        userdata.recognized_word = self.word

        return 'succeeded'


    def recognitionCB(self,recognition):
        REMOVE_LIST = [' huh ',' hum ', ' wa ', ' sh ', ' ch ', ' s ', ' mm ',' pu ', ' tu ', ' ss ',' m ',' h ',' naa ']
        pattern_remove = '|'.join(map(re.escape,REMOVE_LIST))

        if len(recognition.data)>0:
            sentence = " "+recognition.data+" "
            self.timeslast = time.time()
            l = re.findall(pattern_remove, sentence)
            sentence = recognition.data.strip()
            if len(l) == 0 and sentence.find(" "):
                self.word = sentence
                rospy.loginfo('Bender heard: ' + self.word)

    def recognitionP(self,recognition):
        REMOVE_LIST = [' huh ',' hum ', ' wa ', ' sh ', ' ch ', ' s ', ' mm ',' pu ', ' tu ', ' ss ',' m ',' h ',' naa ']
        pattern_remove = '|'.join(map(re.escape,REMOVE_LIST))


        if len(recognition.data)>0:
            sentence = " "+recognition.data+" "
            self.timeslastP = time.time()
            l = re.findall(pattern_remove, sentence)
            sentence = recognition.data.strip()
            if len(l) == 0 and sentence.find(" ") and len(sentence)> len(self.wordP):
                self.wordP = sentence
                rospy.loginfo('Bender heard partial: ' + self.wordP)
    


def getInstance():

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys=['dictionary'],
                            output_keys=['recognized_word'])

    with sm:
        smach.StateMachine.add('RECOGNIZE',Recognize(),
               transitions={'succeeded':'succeeded',
                            'preempted':'RECOGNIZE'},
               remapping={'dictionary':'dictionary',
                          'recognized_word':'recognized_word'})
    return sm


if __name__ == '__main__':

    rospy.init_node('Recognize')

    sm = getInstance()
    ud = smach.UserData()
    ud.dictionary = "gpsr_c2"

    # introspection server
    sis = smach_ros.IntrospectionServer('Recognize', sm, '/RECOGNIZE_SM ')
    sis.start()
    outcome = sm.execute(ud)
    sis.stop()
