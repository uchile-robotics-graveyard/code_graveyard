#!/usr/bin/env python
'''
bool recognize_question_server(bender_srvs::stringReturn::Request &req, bender_srvs::stringReturn::Response &res)
{
	std::string answer;
	std::string question;
	std::string keyword;
	ros::Rate loop_rate(1);
'''

import roslib
import rospy
roslib.load_manifest('bender_speech')

import smach
import smach_ros
from bender_srvs.srv import ObjectDetection
from datetime import datetime

from bender_macros.speech import Recognize
from bender_macros.speech import Talk

#Set number of questions available
number_of_questions = 0
recognizable_keywords = []
questions = []
answers = []
	
def GetSentences():
	global number_of_questions
	global recognizable_keywords
	global questions
	global answers

	number_of_questions = rospy.get_param('n_questions')

	get_sentences = rospy.ServiceProxy("throw_sentences", ObjectDetection)
	rospy.wait_for_service('throw_sentences')

	sentences = get_sentences()

	recognizable_keywords = sentences.type
	questions = sentences.name
	answers = sentences.detector

	#print recognizable_keywords, questions, answers
	rospy.loginfo('Questions OK, ready for recognition')


class Setup(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded'],
			output_keys = ['recognizable_keywords', 'question', 'answer']
		)
		self.recognizable_keywords = []
		self.question = []
		self.answer = []

	def execute(self, userdata):
		#Download variables
		#get_parameters = rospy.ServiceProxy("/bender/speech/interaction/get_param", ObjectDetection)
		rospy.wait_for_service('/bender/speech/interaction/get_param')
		for i in range(number_of_questions):
			self.recognizable_keywords[i] = rospy.get_param('/bender/speech/interaction/get_param').type[i]
			self.question = rospy.get_param('/bender/speech/interaction/get_param').name[i]
			self.answer = rospy.get_param('/bender/speech/interaction/get_param').detector[i]
			
		return 'succeeded'


class findKeyword(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded','aborted'],
			input_keys = ['recognized_word','recognizable_keywords'],
			output_keys = ['recognized_keyword']
		)

	def execute(self, userdata):
		#Find the number of the question
		for i in range(len(recognizable_keywords)):
			if userdata.recognized_word == userdata.recognizable_keywords[i]:
				userdata.recognized_keyword = i
				return 'succeeded'
		return 'aborted'

class findQuestion(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded','preempted','aborted'],
			input_keys = ['recognized_keyword', 'question']
		)

	def execute(self, userdata):
		#Print recognized question
		userdata.recognized_question = userdata.question[userdata.recognized_keyword]
		rospy.loginfo('Recognized Question: ' + userdata.recognized_question)
		return 'succeeded'

class findAnswer(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['succeeded'],
			input_keys = ['recognized_keyword', 'answer'],
			output_keys = ['recognized_answer'])

	def execute(self, userdata):
		#Find answer 
		userdata.recognized_answer = userdata.answer[userdata.recognized_keyword]
		rospy.loginfo('Recognized answer: ' + userdata.recognized_answer)
		if userdata.recognized_answer == 'time':
			hour    = datetime.now().strftime('%H')
			minutes = datetime.now().strftime('%M')
			userdata.recognized_answer = 'The time is '+ hour + ' with ' + minutes + ' minutes'
			return 'succeeded'
		return 'succeeded'

def getInstance():

	sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])

	#Listas disponibles
	sm.userdata.recognizable_keywords = []
	sm.userdata.question = []
	sm.userdata.answer = []

	#Variables
	sm.userdata.recognized_word = ''
	sm.userdata.recognized_keyword = 0
	sm.userdata.recognized_question = ''
	sm.userdata.recognized_answer = ''
	sm.userdata.timeout = 2

	with sm:

		smach.StateMachine.add('RECOGNIZE', Recognize.getInstance(),
			transitions = {'succeeded':'FIND_KEYWORD'},
			remapping = {'recognized_word':'recognized_word'}
			)

		smach.StateMachine.add('FIND_KEYWORD', findKeyword(),
			transitions = {'succeeded':'FIND_QUESTION', 'aborted':'RECOGNIZE'},
			remapping = {'recognized_word':'recognized_word', 'recognized_keyword':'recognized_keyword'}
			)

		smach.StateMachine.add('FIND_QUESTION', findQuestion(),
			transitions = {'succeeded':'FIND_ANSWER', 'preempted':'FIND_QUESTION'},
			remapping = {'recognized_keyword':'recognized_keyword'}
			)

		smach.StateMachine.add('FIND_ANSWER', findAnswer(),
			transitions = {'succeeded':'succeeded'},
			remapping = {'recognized_keyword':'recognized_keyword', 'recognized_answer':'recognized_answer'}
			)

#		smach.StateMachine.add('TALK', talk(),
#			transitions = {'succeeded':'succeeded'},
#			remapping = {'recognized_answer':'recognized_answer','timeout':'timeout'}
#			)
	return sm

if __name__ == '__main__':
	
	rospy.init_node('InteractSpeech')

	GetSentences()
	sm = getInstance()



