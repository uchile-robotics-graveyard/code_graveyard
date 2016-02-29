/*
 * AskQuestion.cpp
 *
 *  Created on: 13-05-2014
 *      Author: matias
 */

#include "bender_speech/macros/AskQuestion.h"

// TODO: usar un campo: 'hints', que indique que palabras debe contener
// si o si la respuesta!. Util en caso de que el diccionario usado tenga
// otras frases que no sirven para la pregunta en cuestión.

namespace bender_speech {

	AskQuestion::AskQuestion(std::string name): _name(name) {

		ros::NodeHandle priv("~");

		_tts_client = priv.serviceClient<bender_srvs::synthesize>("/bender/speech/synthesizer/synthesize");
		_rec_start_client = priv.serviceClient<std_srvs::Empty>("/bender/speech/recognizer/start");
		_rec_stop_client = priv.serviceClient<std_srvs::Empty>("/bender/speech/recognizer/stop");
		_rec_load_client = priv.serviceClient<bender_srvs::load_dictionary_service>("/bender/speech/recognizer/load_dictionary");

		while ( ros::ok() && !_tts_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_rec_start_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_rec_stop_client.waitForExistence(ros::Duration(3.0)) );
		while ( ros::ok() && !_rec_load_client.waitForExistence(ros::Duration(3.0)) );
		
		_rec_output_subscriber = priv.subscribe("/bender/speech/recognizer/output",1,&AskQuestion::speech_rec_cb,this);

		_question_server = priv.advertiseService("ask", &AskQuestion::ask,this);
		_question_server2 = priv.advertiseService("askConfirm", &AskQuestion::askConfirm,this);

		ROS_INFO_STREAM("Working . . .");
	}

	AskQuestion::~AskQuestion() {}

	bool AskQuestion::askConfirm(bender_srvs::AskQuestion::Request &req, bender_srvs::AskQuestion::Response &res) {

		// load dictionary
		if (!load_dictionary(req.dictionary)) {
			return false;
		}

		// start recognition
		start_recognition();

		// ask question
		talk(req.question, req.qx_sleep);
		_rec_output = "";

		while (ros::ok()) {

			ros::Duration(0.5).sleep();
			std::size_t found_yes = _rec_output.find("yes");
			std::size_t found_no = _rec_output.find("no");

			ROS_WARN_STREAM("current state: "
					<< "_rec_output='" << _rec_output
					<< "', found_yes_pos:'" << found_yes
					<< "', found_no_pos:'" << found_no
					<< "', npos:" << std::string::npos);

			// check yes
			if (found_yes != std::string::npos) {

				stop_recognition();
				res.answer = "yes";
				return true;
			}

			// check no
			if (found_no != std::string::npos) {

				stop_recognition();
				res.answer = "no";
				return true;
			}
		}

		return false;
	}

	bool AskQuestion::ask(bender_srvs::AskQuestion::Request &req, bender_srvs::AskQuestion::Response &res) {

		std::string answer;
		int max_attempts = req.max_attempts;
		int attempts = 0;

		if (!load_dictionary(req.dictionary)) {
			return false;
		}

		bool response_confirmed = false;

		while(attempts < max_attempts) {

			attempts++;

			talk(req.question);
			start_recognition();
			_rec_output = "-1";
			while (_rec_output == "-1" && ros::ok()) {
				ros::Duration(0.5).sleep();
			}
			answer = _rec_output;
			stop_recognition();

			if (confirmResponse(answer)) {

				res.answer = answer;
				return true;
			}
		}

		// max attempts limit reached
		res.answer = "max_attempts_reached";
		return true;
	}

	bool AskQuestion::confirmResponse(std::string response) {

		std::string answer;

		load_dictionary("confirmation");
		talk("Did you say. " + response + " ?");
		start_recognition();
		_rec_output = "-1";
		while (_rec_output == "-1" && ros::ok()) {
			ros::Duration(0.5).sleep();
		}
		answer = _rec_output;
		stop_recognition();

		if (answer == "yes") {
			return true;
		}

		return false;
	}

	void AskQuestion::speech_rec_cb(std_msgs::String msg) {

		ROS_WARN_STREAM("rec cb: " << msg);
		_rec_output = msg.data;
	}

	void AskQuestion::talk(std::string text, float sleep_time) {

		ROS_INFO_STREAM("[" << _name << "]: Talking: '" << text << "'");

		bender_srvs::synthesize tts_srv;
		tts_srv.request.text = text;
		while ( ros::ok() && !_tts_client.waitForExistence(ros::Duration(3.0)) );
		_tts_client.call(tts_srv);

		/*
		 * TODO: REPARAR!, se queda pegado :'(
		ros::NodeHandle n;
		bool _is_talking = false;
		while (!_is_talking) {
			ros::Duration(0.5).sleep();
			n.getParam("/bender/speech/synthesizer/talking",_is_talking);
			//ROS_DEBUG_STREAM("not talking . . .");
		}
		while(_is_talking) {
			ros::Duration(0.5).sleep();
			n.getParam("/bender/speech/synthesizer/talking",_is_talking);
			//ROS_DEBUG_STREAM("talking . . .");
		}*/
		ros::Duration(sleep_time).sleep();

	}

	void AskQuestion::start_recognition() {

		// start recognition
		std_srvs::Empty start_srv;
		while ( ros::ok() && !_rec_start_client.waitForExistence(ros::Duration(3.0)) );
		_rec_start_client.call(start_srv);
	}

	void AskQuestion::stop_recognition() {

		// end recognition
		std_srvs::Empty stop_srv;
		while ( ros::ok() && !_rec_stop_client.waitForExistence(ros::Duration(3.0)) );
		_rec_stop_client.call(stop_srv);
	}

	bool AskQuestion::load_dictionary(std::string dictionary) {

		bender_srvs::load_dictionary_service load_srv;
		load_srv.request.dictionary = dictionary;
		while ( ros::ok() && !_rec_load_client.waitForExistence(ros::Duration(3.0)) );
		_rec_load_client.call(load_srv);

		if( load_srv.response.output.find("not found") < load_srv.response.output.size()) {
			ROS_WARN_STREAM("Dictionary: '" << dictionary << "' not found!");
			return false;
		}

		return true;
	}

} /* namespace bender_speech */

int main(int argc, char **argv)
{
	ros::init(argc,argv,"ask_question");

	bender_speech::AskQuestion *node = new bender_speech::AskQuestion(ros::this_node::getName());

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	ROS_INFO("Quitting ... \n");
	return 0;
}
