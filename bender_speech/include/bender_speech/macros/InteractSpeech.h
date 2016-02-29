/*
 * InteractSpeech.h
 *
 *  Created on: Jan 21, 2014
 *      Author: gonzaloolave
 */

#ifndef INTERACTSPEECH_H_
#define INTERACTSPEECH_H_

#include "bender_srvs/load_dictionary_service.h"
#include "bender_srvs/synthesize.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "bender_srvs/TimerString.h"
#include <bender_srvs/ObjectDetection.h>

#include <list>
#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include "bender_srvs/Dummy.h"
#include <ctime>
#include <sys/timeb.h>
using std::string;
using std::stringstream;

using namespace std;

#define recognizable_drinks_size 8
#define recognizable_names_size 20
#define NUMBER_OF_QUESTIONS 47

ros::NodeHandle * n;
bool _is_talking;

bool recognize_question_server(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res);
bool recognize_drink(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res);
bool interaction_speech(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res);

void synthesizer_status_callback(std_msgs::String status);
void recognizer_callback(std_msgs::String rec);
void start_interaction();
//void recognize_drink_request();
void talk(std::string sentence);
void findKey(string stream);
void findKey2(string stream);
string splitMsg(string stream);
void downloadVariables();
bool confirm_word(std::string word);
std::string findQuestion(std::string key);
std::string findAnswer(std::string key);


ros::ServiceServer interact;
ros::ServiceServer recognize;
ros::ServiceServer quest;
ros::ServiceServer quest2;
ros::ServiceServer quest_end;
ros::ServiceClient recon;
ros::ServiceClient start_recon;
ros::ServiceClient stop_recon;
ros::ServiceClient start_recon2;
ros::ServiceClient stop_recon2;
ros::ServiceClient talk_srv;
ros::ServiceClient get_parameters;

ros::Subscriber recon_sub;
ros::Subscriber recon_sub2;
ros::Subscriber talk_sub;

int number_of_questions;
std::string synthesizer_status;
std::string recognized_keyword = "none";
std::string recognized_question = "none";
std::string recognized_answer = "none";
std::string recognized_drink = "none";
std::string recognized_name = "none";
std::string confirmation = "none";

std::string answers[47];
std::string recognizable_keywords[47];
std::string questions[47];

std::string last_dictionary;

// Si se modifica el diccionario agregar las palabras aca y modificar las variables recognizable_drinks_size y recognizable_names_size
const char * recognizable_drinks[] = {"cola","beer","chocolate milk","energy drink","grape juice","milk","orange juice","water"};
const char * recognizable_names[] = {"anna","beth","carmen","jennifer","jessica","kimberly","kristina","laura","mary","sarah","alfred","charles","daniel","james","john","luis","paul","richard","robert","steve"};


#endif /* INTERACTSPEECH_H_ */
