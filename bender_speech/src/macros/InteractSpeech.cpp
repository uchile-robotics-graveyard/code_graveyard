/*
 * InteractSpeech.cpp
 *
 *  Created on: Jan 21, 2014

 *      Author: bendervision
 */

#include "bender_speech/macros/InteractSpeech.h"
#include <boost/algorithm/string.hpp>


void synthesizer_status_callback(std_msgs::String status)
{
	synthesizer_status = status.data.c_str();
}

void recognizer_callback(std_msgs::String rec)
{
	std::string msg = rec.data.c_str();
	std::string drnk;
	std::string name;
	//ROS_WARN("%s\n",msg.c_str());
	//string only_name = "my name is anna";

	if (msg.compare("wa")!=0||msg.compare("hum")!=0||msg.compare("sh")!=0||msg.compare("ch")!=0||msg.compare("s")!=0||msg.compare("mm")!=0){
		int last = msg.find_last_of(' ');
		std::cout<<msg.find("name");
		if(msg.find("name")!=std::string::npos)
				name = rec.data.substr(last+1);
		//ROS_WARN("%s\n",name.c_str());
		if(msg.find("want")!=std::string::npos)
				drnk = rec.data.substr(7);
		//drnk = rec.data.substr(last+1);
		//ROS_WARN("%s\n",drnk.c_str());
	}
	ROS_INFO("Bender heard: %s",rec.data.c_str());
	for (int i = 0; i < recognizable_drinks_size; i++){
		if(drnk.compare(recognizable_drinks[i]) == 0)
			recognized_drink = recognizable_drinks[i];
	}
	for  (int i = 0; i < recognizable_names_size; i++){
		if(name.compare(recognizable_names[i]) == 0)
			recognized_name = recognizable_names[i];
	}

	string parsed_msg = splitMsg(msg);

	findKey(parsed_msg);

	if(msg.compare("bender yes") == 0){
		confirmation = "yes";
	}
	else if (msg.compare("bender no") == 0){
		confirmation = "no";
	}
}

void recognizer_callback2(std_msgs::String rec)
{
	std::string msg = rec.data.c_str();
	std::string drnk;
	std::string name;

	if (msg.compare("wa")!=0||msg.compare("hum")!=0||msg.compare("sh")!=0||msg.compare("ch")!=0||msg.compare("s")!=0||msg.compare("mm")!=0)
	{
		int last = msg.find_last_of(' ');
		std::cout<<msg.find("name");
		if(msg.find("name")!=std::string::npos)
				name = rec.data.substr(last+1);
		if(msg.find("want")!=std::string::npos)
				drnk = rec.data.substr(7);
	}

	ROS_INFO("Bender heard: %s",rec.data.c_str());
	string parsed_msg = splitMsg(msg);

	findKey(parsed_msg);
}

string splitMsg(string stream)
{

	istringstream iss(stream);
	string output;
	do
    {
        string sub,long_word;
        iss >> sub;
        ros::param::get("/bender/speech/interaction/"+sub,long_word)?
        	output += long_word + " " :
        	output += sub + " ";

    } while (iss);
    return output;
}

void findKey(string stream)
{
	std::size_t found;
	int cont = 0;

	for (int i = 0; i < number_of_questions; i++)
	{
		std::vector<std::string> keys;
		std::string s = recognizable_keywords[i].c_str(),tmp=stream;
		boost::split(keys, s, boost::is_any_of(" "), boost::token_compress_on);
		istringstream keywords(recognizable_keywords[i].c_str());
		//ROS_WARN("-----------------");
		//ROS_INFO("  keys: %s %s",keys[0].c_str(), keys[1].c_str());
		//ROS_INFO("stream: %s",stream.c_str());

		int limit = keys.size();
		for (std::vector<std::string>::iterator key = keys.begin(); key != keys.end(); ++key)
		{
			found = tmp.find(*key);
			if(found != -1)
			{
				//ROS_INFO("found: %s at %zd ", key->c_str(), found);	
				size_t len = key->length();
				tmp = stream.substr(found+len+1);
				cont++;
			}
		}	 
		if(cont >= limit){
			//ROS_INFO("cont %d", cont);
			recognized_question = questions[i];
			recognized_answer = answers[i];
			break;
		}
		cont = 0;
	}
}


void talk(std::string text) {

	ROS_INFO_STREAM("[InteractSpeech]: Talking: " << text);

	bender_srvs::synthesize tts_srv;
	tts_srv.request.text = text;
	while ( ros::ok() && !talk_srv.waitForExistence(ros::Duration(3.0)) );
	talk_srv.call(tts_srv);

	ros::Duration(3.5).sleep();

}

bool interaction_speech(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res)
{
	std_srvs::Empty empty;
	std::string nombre;
	ros::Rate loop_rate(1);
	bool yesorno = false;

	bender_srvs::load_dictionary_service msg;
	msg.request.dictionary = "name";
	last_dictionary = msg.request.dictionary;
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");
	else ROS_INFO("Dictionary '%s' loaded",msg.request.dictionary.c_str());


	while (!yesorno){

		talk("Please tell me your name");		
		if(!start_recon.call(empty))
			ROS_ERROR("Failed to start recognizing");
		ROS_INFO("START");


		while(recognized_name == "none" && ros::ok())
		{
			ROS_INFO("Bender hasn't recognized any name");
			loop_rate.sleep();
			ros::spinOnce();
		}

		if(!stop_recon.call(empty))
			ROS_ERROR("Failed to stop recognizing");

		ROS_INFO("Recognized name: %s", recognized_name.c_str());

		nombre = recognized_name;
		recognized_name = "none";
		yesorno = confirm_word(nombre);
	}

	std::string saludo = "hello, " + nombre;
	talk(saludo);

	res.data = nombre.c_str();
	return true;
}

bool recognize_drink(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res) {
	
	std::string drink;
	ros::Rate loop_rate(1);
	bool yesorno = false;

	bender_srvs::load_dictionary_service msg;
	msg.request.dictionary = "drink";
	last_dictionary = msg.request.dictionary;
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");
	else ROS_INFO("Dictionary '%s' loaded",msg.request.dictionary.c_str());

	while (!yesorno){

		talk("what drink do you want?");

		std_srvs::Empty empty;
		if(!start_recon.call(empty))
			ROS_ERROR("Failed to start recognizing");

		while(recognized_drink == "none" && ros::ok())
		{
			ROS_INFO("Bender hasn't recognized any drink");
			loop_rate.sleep();
			ros::spinOnce();
		}

		if(!stop_recon.call(empty))
			ROS_ERROR("Failed to stop recognizing");

		ROS_INFO("Recognized drink: %s", recognized_drink.c_str());

		drink = recognized_drink;
		recognized_drink = "none";

		yesorno = confirm_word(drink);
	}

	res.data = drink.c_str();
	return true;
}

bool confirm_word(std::string word)
{
	ros::Rate loop_rate(1);
	bender_srvs::load_dictionary_service msg;
	msg.request.dictionary = "confirmation";
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");
	else ROS_INFO("Dictionary '%s' loaded",msg.request.dictionary.c_str());

	std::string speech = "did you say, " + word;
	talk(speech);

	std_srvs::Empty empty;
	if(!start_recon.call(empty))
		ROS_ERROR("Failed to start recognizing");

	while(confirmation == "none" && ros::ok())
	{
		ROS_INFO("Bender hasn't recognized any word");
		loop_rate.sleep();
		ros::spinOnce();
	}

	if(!stop_recon.call(empty))
		ROS_ERROR("Failed to stop recognizing");

	msg.request.dictionary = last_dictionary;
	if(!recon.call(msg))
		ROS_ERROR("Failed to load dictionary");
	else ROS_INFO("Dictionary '%s' loaded",msg.request.dictionary.c_str());

	ROS_INFO("Confirmation: %s", confirmation.c_str());

	if (confirmation == "yes"){
		confirmation = "none";
		return true;
	}
	else if (confirmation == "no"){
		confirmation = "none";
		return false;
	}

	return false;
}


bool recognize_question_server(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res)
{
	std::string answer;
	std::string question;
	std::string keyword;
	int cont = 0;
	ros::Rate loop_rate(1);

	//talk("I will listen after the tone");
	std_srvs::Empty empty;
	if(!start_recon.call(empty))
		ROS_ERROR("Failed to start recognizing");

	while(recognized_question == "none" && ros::ok())
	{
		ROS_INFO("Bender hasn't recognized any keyword");
		loop_rate.sleep();
		ros::spinOnce();
		cont++;
		if (cont == req.timeout){
			res.timeout = true;
			stop_recon.call(empty);
			return true;
		}
	}

	if(!stop_recon.call(empty))
		ROS_ERROR("Failed to stop recognizing");

	question = recognized_question;
	recognized_question = "none";

	talk("you say "+question);

	if (answer == "time"){
		std::time_t now = time(0);
		 tm *ltm = localtime(&now);
		 std::cout << "Time: "<< 1 + ltm->tm_hour << ":";
		   std::cout << 1 + ltm->tm_min << ":";
		int hroa=( 1 + ltm->tm_hour);
		int min=1 + ltm->tm_min;

		std::stringstream tin;
		tin<< " It is "<<hroa<< " with "<<min<<" minutes";
		answer=tin.str().c_str();
	}
	talk(recognized_answer.c_str());


	res.data = answer.c_str();
	res.timeout = false;
	return true;
}

bool recognize_question_server2(bender_srvs::TimerString::Request &req, bender_srvs::TimerString::Response &res)
{
	std::string answer;
	std::string question;
	std::string keyword;
	int cont = 0;
	ros::Rate loop_rate(1);

	//talk("I will listen after the tone");
	std_srvs::Empty empty;
	if(!start_recon2.call(empty))
		ROS_ERROR("Failed to start recognizing");

	while(recognized_question == "none" && ros::ok())
	{
		ROS_INFO("Bender hasn't recognized any keyword");
		loop_rate.sleep();
		ros::spinOnce();
		cont++;
		if (cont == req.timeout){
			res.timeout = true;
			stop_recon.call(empty);
			return true;
		}
	}

	if(!stop_recon2.call(empty))
		ROS_ERROR("Failed to stop recognizing");

	question = recognized_question;
	recognized_question = "none";

	talk("you say "+question);

	if (answer == "time"){
		std::time_t now = time(0);
		 tm *ltm = localtime(&now);
		 std::cout << "Time: "<< 1 + ltm->tm_hour << ":";
		   std::cout << 1 + ltm->tm_min << ":";
		int hroa=( 1 + ltm->tm_hour);
		int min=1 + ltm->tm_min;

		std::stringstream tin;
		tin<< " It is "<<hroa<< " with "<<min<<" minutes";
		answer=tin.str().c_str();
	}
	talk(recognized_answer.c_str());


	res.data = answer.c_str();
	res.timeout = false;
	return true;
}


void downloadVariables(){

	bender_srvs::ObjectDetection param_request;
	ros::param::get("n_questions",number_of_questions);

	//ROS_INFO("Number: %d",number_of_questions);

	if (get_parameters.call(param_request)){

		ROS_INFO("Getting info...");
		for (int i = 0; i<number_of_questions; i++) {
			questions[i] = param_request.response.name[i];
			recognizable_keywords[i] = param_request.response.type[i];
			answers[i] = param_request.response.detector[i];
		}
		ROS_INFO("Variables donwloaded");
	}
	else ROS_ERROR("Couldn't download variables");
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"interaction_macros");

	ros::NodeHandle nh("~");
	n = &nh;

	recon = nh.serviceClient<bender_srvs::load_dictionary_service>("/bender/speech/recognizer/load_dictionary");
	start_recon = nh.serviceClient<std_srvs::Empty>("/bender/speech/recognizer/start");
	stop_recon = nh.serviceClient<std_srvs::Empty>("/bender/speech/recognizer/stop");
	start_recon2 = nh.serviceClient<std_srvs::Empty>("/bender/speech/recognizer2/start");
	stop_recon2 = nh.serviceClient<std_srvs::Empty>("/bender/speech/recognizer2/stop");
	talk_srv = nh.serviceClient<bender_srvs::synthesize>("/bender/speech/synthesizer/synthesize");
	get_parameters = nh.serviceClient<bender_srvs::ObjectDetection>("throw_sentences");

	recon_sub = nh.subscribe("output",1,recognizer_callback);
	recon_sub2 = nh.subscribe("output2",1,recognizer_callback2);
	talk_sub = nh.subscribe("status",1,synthesizer_status_callback);

	quest = nh.advertiseService("answer_request",recognize_question_server);
	quest2 = nh.advertiseService("answer_request2",recognize_question_server2);

	while ( ros::ok() && !get_parameters.waitForExistence(ros::Duration(3.0)) );

	// TODO: use a ("~") nodehandle and names like ("(empty)request_name,...")
	interact = nh.advertiseService("request_name",interaction_speech);
	recognize = nh.advertiseService("request_drink",recognize_drink);

	while ( ros::ok() && !recon.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !start_recon.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !stop_recon.waitForExistence(ros::Duration(3.0)) );
	while ( ros::ok() && !talk_srv.waitForExistence(ros::Duration(3.0)) );
	
	downloadVariables();

	ros::spin();

	ROS_INFO("Quitting ... \n");
	return 0;

}

