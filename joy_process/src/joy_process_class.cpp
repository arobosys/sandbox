#include "joy_process_class.hpp"
#include "sensor_msgs/Joy.h"
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <malish/Diode.h>
#include <tf/tf.h>

JoyProcess::JoyProcess(ros::NodeHandle &handle, int argc, char **argv) {

    bindbuttons[0] = false;
	bindbuttons[1] = false;
	_restart = false;
	goalisactiv = false;

	joystik = handle.subscribe("/joy", 10, &JoyProcess::joyCommandCallback, this);

    led_pub_ = handle.advertise<malish::Diode>("/led", 10);

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

	bindbuttons[2] = command.buttons[2];
	bindbuttons[1] = command.buttons[1];

	if(goalisactiv && ((bindbuttons[2] == true)||(bindbuttons[1] == true))) {

        ROS_INFO("JoyCMD: button A %d ", bindbuttons[2] );
        ROS_INFO("JoyCMD: button B %d ", bindbuttons[1] );
		goalisactiv = false;

		ProcessInterface::Result alibResultOk;

		ProcessInterface::Feedback feedback;

		core_msgs::DataPL dataPL;

		dataPL.command = "execute";
		dataPL.id = rosLinkClientPtr->getId();
		dataPL.type = "feedback";

		core_msgs::KeyValue keyValue;
		keyValue.key = "point";

		int button = 0;
		if(bindbuttons[2] == true) {
			keyValue.value = "1";// для кнопки А
            button = 1;
		}
		if (bindbuttons[1] == true) {    //
			keyValue.value = "2";// для кнопки B
            button = 2;
		}
		Pointline k;
		for(int i=0;i<vec_d.size();i++)
        {
        	/*
        	k.id = 0;
			k.x = 1;
			k.y = 1;
			k.w = 90;

        	for(int r=0; r<255; i++ ) {
				for (int g = 0; g < 255; i++) {
					for (int b = 0; b < 255; i++) {
						k.r = r;
						k.g = g;
						k.b = b;
						setpointcolor(k);
					}
				}
			}*/
            if(vec_d[i].id == button) {
				setpointcolor(vec_d[i]);//	ROS_INFO("JoyCMD: PointNumber %d ", p.id );
				sendgoal(vec_d[i]);
			}
        };

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

void JoyProcess::sendgoal(Pointline p)
{
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(10.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = p.x;
	goal.target_pose.pose.position.y = p.y;

    double radians = p.w * (M_PI/180);
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("SUCCEEDED");
	else
		ROS_INFO("UNSUCCEEDED");
}

void  JoyProcess::setpointcolor(Pointline p)
{
	malish::Diode led_msg_;

	led_msg_.red = p.r;
	led_msg_.green = p.g;
	led_msg_.blue = p.b;

	led_pub_.publish(led_msg_);
}

void  JoyProcess::setPointByNum(std::string num)
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
        if(vec_d[i].id == std::stoi(num)) {
            setpointcolor(vec_d[i]);//	ROS_INFO("JoyCMD: PointNumber %d ", p.id );
            sendgoal(vec_d[i]);
        }
    };
}