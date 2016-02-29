/*
 * Sentences.cpp
 *
 *  Created on: 6 Feb 2014
 *      Author: gonzaloolave
 */

#include "bender_speech/macros/Sentences.h"

namespace bender_macros {

Sentences::Sentences(int _ids) {
	// TODO Auto-generated constructor stub
	ros::NodeHandle nh("~");
	n = &nh;

	number = 0;

	_id = new int[_ids];
	_answers = new string[_ids];
	_questions = new string[_ids];
	_keywords = new string[_ids];

	_write_question = n->advertiseService("insert_question",&Sentences::insert_question_server,this);
	_ask_question = n->advertiseService("ask_question",&Sentences::ask_question_server,this);

	_param_service = n->advertiseService("throw_sentences",&Sentences::get_parameters_server,this);

	_files_path = ros::package::getPath("bender_speech") + "/config/Sentences/";

}

Sentences::~Sentences() {
	// TODO Auto-generated destructor stub
}

// functions
bool Sentences::insert_question_server(bender_srvs::QuestionService::Request &req, bender_srvs::QuestionService::Response &res){

	res.answer = this->question_receiver(req.choice,req.question,req.keyword,req.answer);
	return true;
}

bool Sentences::ask_question_server(bender_srvs::QuestionService::Request &req, bender_srvs::QuestionService::Response &res){

	res.answer = this->question_receiver(req.choice,req.question,req.keyword,req.answer);
	return true;
}

bool Sentences::get_parameters_server(bender_srvs::ObjectDetection::Request &req, bender_srvs::ObjectDetection::Response &res){

	res.detector.resize(number);
	res.name.resize(number);
	res.type.resize(number);

	//ROS_WARN("number: %d", number);
	for ( int i=0 ; i<number ; i++){
		res.name[i] = this->_questions[i];
		res.type[i] = this->_keywords[i];
		res.detector[i] = this->_answers[i];
	}

	return true;
}

string Sentences::question_receiver(int choice, string _quest, string _key, string _ans){
	string answer;
	if (choice == 0){ // insert question, keyword and answer to the database
		this->setQuestion(number,_quest);
		this->setKeywords(number,_key);
		this->setAnswer(number,_ans);
		this->UploadVariables();
	}
	else if (choice == 1){	// asks question from database and wait for the answer
		int ID = 0;
		ID = this->getId(_key);
		if ( ID == -1 ){
			answer = "please give a keyword";
		}
		else{
			answer = this->getAnswer(ID);
		}
	}
	return answer;
}

string Sentences::getAnswer(int id){

	return this->_answers[id];
}

std::string Sentences::getQuestion(int id){

	return this->_questions[id];
}

std::string Sentences::getKeywords(int id){

	return this->_keywords[id];
}

int Sentences::getId(string keywords){
	int resultado = 0;
	for (int i = 0; i<number ; i++){
		if ( keywords.compare(this->_keywords[i]) == 0){
			ROS_WARN("key: %s",this->_keywords[i].c_str());
			resultado = i;
			return resultado;
		}
	}

	return -1;
}

void Sentences::setAnswer(int id, string ans){

	string ans_path = _files_path + "answers.txt";
	_answer.open(ans_path.c_str(), ofstream::app);

	char to_write[100];
	sprintf(to_write,"%d %s\n",id,ans.c_str());
	_answer << to_write;

	_answer.close();
}

void Sentences::setQuestion(int id, string quest){
	
	string quest_path = _files_path + "questions.txt";
	_question.open(quest_path.c_str(), ofstream::app);

	char to_write[100];
	sprintf(to_write,"%d %s\n",id,quest.c_str());
	_question << to_write;

	_question.close();
}

void Sentences::setKeywords(int id, string key){

	string key_path = _files_path + "keywords.txt";
	_keyword.open(key_path.c_str(), ofstream::app);

	char to_write[100];
	sprintf(to_write,"%d %s\n",id,key.c_str());
	_keyword << to_write;

	_keyword.close();
}

void Sentences::UploadVariables(){

	string line = "";
	int cont = 0;
	unsigned zero = 0;
	string asd;

	string key_path = _files_path + "keywords.txt";
	ifstream file_key (key_path.c_str());
	string ans_path = _files_path + "answers.txt";
	ifstream file_answer (ans_path.c_str());
	string quest_path = _files_path + "questions.txt";
	ifstream file_quest (quest_path.c_str());

	//ROS_WARN("requesting...");
	while ( getline(file_quest,line) && cont <= NUMBER_OF_QUESTIONS){
		//ROS_INFO("Line %d: '%s'",cont,line.c_str());
		this->_questions[cont] = line;
		cont++;
	}
	cont = 0 ;
	line = "";
	while ( getline(file_answer,line) && cont <= NUMBER_OF_QUESTIONS){
		
		this->_answers[cont] = line;
		cont++;
	}
	cont = 0;
	line = "";
	while ( getline(file_key,line) && cont <= NUMBER_OF_QUESTIONS){
		
		this->_keywords[cont] = line;
		//ROS_WARN("CONT: %d",cont);
		cont++;
	}
	number = cont;
	ros::param::set("n_questions",number);

	//	ROS_WARN("... ...");
	ROS_INFO("variables uploaded, number of lines: %d",number);
}

} /* namespace bender_macros */


int main(int argc, char **argv){

	ros::init(argc,argv,"speech_sentences");

	bender_macros::Sentences *sentence = new bender_macros::Sentences(NUMBER_OF_QUESTIONS);

	sentence->UploadVariables();

	ros::spin();

	return 0;
}
