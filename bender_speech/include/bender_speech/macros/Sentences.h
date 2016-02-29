/*
 * Sentences.h
 *
 *  Created on: 6 Feb 2014
 *      Author: gonzaloolave
 */

#ifndef SENTENCES_H_
#define SENTENCES_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <fstream>

#include <bender_srvs/ObjectDetection.h>
#include <bender_srvs/QuestionService.h>

#define NUMBER_OF_QUESTIONS 47

using std::ofstream;
using std::ifstream;
using std::string;

namespace bender_macros {

class Sentences {

public:
	Sentences(int _ids);
	virtual ~Sentences();

	bool insert_question_server(bender_srvs::QuestionService::Request &req, bender_srvs::QuestionService::Response &res);
	bool ask_question_server(bender_srvs::QuestionService::Request &req, bender_srvs::QuestionService::Response &res);
	bool get_parameters_server(bender_srvs::ObjectDetection::Request &req, bender_srvs::ObjectDetection::Response &res);

	string getAnswer(int id);
	string getQuestion(int id);
	string getKeywords(int id);

	void setAnswer(int id, string ans);
	void setQuestion(int id, string quest);
	void setKeywords(int id, string key);

	string question_receiver(int choice, string _quest, string _key, string _ans);
	int getId(string keywords);

	void UploadVariables();

private:

	ros::NodeHandle * n;
	ofstream _question,_answer,_keyword;

	int number;

	int* _id;
	string* _answers;
	string* _questions;
	string* _keywords;

	string _files_path;

	ros::ServiceServer _write_question;
	ros::ServiceServer _ask_question;
	ros::ServiceServer _param_service;

};

} /* namespace bender_macros */
#endif /* SENTENCES_H_ */
