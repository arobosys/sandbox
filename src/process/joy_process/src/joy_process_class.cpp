#include "joy_process_class.hpp"
#include "sensor_msgs/Joy.h"


#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



//#include "Diode.h"




JoyProcess::JoyProcess(ros::NodeHandle &handle, int argc, char **argv) {

	bindbuttons[0] = false;
	bindbuttons[1] = false;
	_restart = false;
	goalisactiv = false;

	joystik = handle.subscribe("/joy", 10, &JoyProcess::joyCommandCallback, this);

	if(!joystik)
		ROS_WARN("Cannot connect to joystik topic");
	else
		ROS_INFO("joy_process_callback  is set");
}

int JoyProcess::setmode(const int mode){
	JoyProcess::_mode = mode;
	return 0;
}

void JoyProcess::joyCommandCallback(const sensor_msgs::Joy command)
{


	std::ifstream ifs{"pointers.txt"};
	std::vector<Pointline> vec_d;
	Pointline p;

	while(ifs >> p.id >> p.x >> p.y >> p.w >> p.r >> p.g >> p.b)
	{

		vec_d.push_back(p);
	}

	for(int i=0;i<vec_d.size();i++)
	{
	//	std::cout << vec_d[i].x<< " " << vec_d[i].y << " " <<vec_d[i].w << " " << vec_d[i].g << std::endl;

		ROS_INFO("JoyCMD: PointNumber %d ", p.id );

	};






	bindbuttons[0] = command.buttons[0];
	bindbuttons[1] = command.buttons[1];

	ROS_INFO("JoyCMD: button A %d ", bindbuttons[0] );
	ROS_INFO("JoyCMD: button B %d ", bindbuttons[1] );

	if(goalisactiv && ((bindbuttons[0] == true)||(bindbuttons[1] == true))) {

		goalisactiv = false;

		ProcessInterface::Result alibResultOk;

		ProcessInterface::Feedback feedback;

		core_msgs::DataPL dataPL;

		dataPL.command = "execute";
		dataPL.id = rosLinkClientPtr->getId();
		dataPL.type = "feedback";

		core_msgs::KeyValue keyValue;
		keyValue.key = "point";

		if(bindbuttons[0] == true) {
			keyValue.value = "1";// для кнопки А
		}
		if (bindbuttons[1] == true) {    //
			keyValue.value = "2";// для кнопки B
		}

		// "1" | "2"  результат нажатия кнопок
		ROS_INFO("Status: %s", keyValue.value.c_str());

		dataPL.states.push_back(keyValue);

		feedback.feedback.push_back(std::move(dataPL));
		rosLinkClientPtr->publishFeedback(std::move(feedback));

		rosLinkClientPtr->setSucceeded(alibResultOk,"OK");
		ROS_INFO("Sended ResultOk");
	}
}

void JoyProcess::SetFeedbackActive( ){
	this->goalisactiv = true;
}
