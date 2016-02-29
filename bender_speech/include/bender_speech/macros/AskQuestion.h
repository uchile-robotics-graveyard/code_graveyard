/*
 * AskQuestion.h
 *
 *  Created on: 13-05-2014
 *      Author: matias
 */

#ifndef ASKQUESTION_H_
#define ASKQUESTION_H_

#include <ros/ros.h>
#include <bender_srvs/AskQuestion.h>
#include <bender_srvs/synthesize.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <bender_srvs/load_dictionary_service.h>

namespace bender_speech {

class AskQuestion {

public:
	ros::ServiceServer _question_server;
	ros::ServiceServer _question_server2;
	ros::ServiceClient _tts_client;
	ros::ServiceClient _rec_start_client;
	ros::ServiceClient _rec_stop_client;
	ros::ServiceClient _rec_load_client;

	ros::Subscriber _rec_output_subscriber;

private:
	std::string _name;
	std::string _rec_output;

public:
	AskQuestion(std::string name);
	virtual ~AskQuestion();

	bool askConfirm(bender_srvs::AskQuestion::Request &req, bender_srvs::AskQuestion::Response &res);
	bool ask(bender_srvs::AskQuestion::Request &req, bender_srvs::AskQuestion::Response &res);
	void speech_rec_cb(std_msgs::String msg);

private:
	void talk(std::string text, float sleep_time = 3.0);
	bool confirmResponse(std::string response);
	void start_recognition();
	void stop_recognition();
	bool load_dictionary(std::string dictionary);


};

} /* namespace bender_speech */
#endif /* ASKQUESTION_H_ */
