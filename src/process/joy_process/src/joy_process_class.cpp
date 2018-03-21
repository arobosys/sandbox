#include "joy_process_class.hpp"

JoyProcess::JoyProcess(ros::NodeHandle &handle, int argc, char **argv) {
	_gogogo = false;
	_restart = false;
    _mode = 0;

	joystik = handle.subscribe("/joy/command", 10, &JoyProcess::joyCommandCallback, this);

		if(!joystik)
			ROS_WARN("Cannot connect to joystik topic");
		else
			ROS_INFO("joy_process_callback  is set");
}

int JoyProcess::setmode(const int mode){
    JoyProcess::_mode = mode;
	return 0;
}

void JoyProcess::joyCommandCallback(const malish::JoyCMD &command) {
    _gogogo = command.gogogo;
    _restart = command.restart;

	ROS_INFO("JoyCMD: gogogo %d reset %d", _gogogo , _restart);

	if(_restart){
		if (feedbackHandler) {
			feedbackHandler();
			ROS_INFO("Send JoyCMD feedback");
		}
	}
}

void JoyProcess::SetFeedback(FeedbackHandler feedbackHandler){
	this->feedbackHandler = feedbackHandler;
}