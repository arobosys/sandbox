#include "joy_process_class.hpp"

JoyProcess::JoyProcess(ros::NodeHandle &handle, int argc, char **argv) {
	_gogogo = false;
	_restart = false;
	joystik = handle.subscribe("/joy/command", 10, &JoyProcess::joyCommandCallback, this);
		if(!joystik)
			ROS_WARN("Cannot connect to joystik topic");
		else
			ROS_INFO("joy_process_callback  is set");
}

void JoyProcess::joyCommandCallback(const malish::JoyCMD &command) {
    _gogogo = command.gogogo;
    _restart = command.restart;
	ROS_INFO("JoyCMD: gogogo %d reset %d", _gogogo , _restart);
}
