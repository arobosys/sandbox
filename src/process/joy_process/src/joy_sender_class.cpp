#include "joy_process.hpp"

JoySender::JoySender(ros::NodeHandle &handle, int argc, char **argv) {
    rosLinkClientPtr = std::make_shared<interface::ProcessInterface>(argc, argv,
                                                                     std::bind(&JoySender::joyCallback, this,
                                                                               std::placeholders::_1),
                                                                     std::bind(&JoySender::preemtCallback,
                                                                               this));
    ROS_INFO("joy_process_callback  is set");
    rosLinkClientPtr->listen();
}


void JoySender::sendInterrupt(const std::map<std::string, std::string> &keyToValue) {
    publisher
      
    std::string goal1_string = keyToValue.at(GOAL1);

    double goalToMove[num_goal][3];

    std::vector<double> goal = splitByDelimiter<double>(goal1_string, ';');
    ROS_INFO("From logic layer I get joy button: y=%f",goal[0]);
    goalToMove[0][0]=goal[0];
    //goal1=goalTransform(goal[1],goal[2],goal[3]);
    goal = splitByDelimiter<double>(goal2_string, ';');

    //tell the action client that we want to spin a thread by default
    JoyClient ac("joy", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for(int i=0;i<num_goal;i++)
    {
        sensor_msgs::Joy joy;

        //The goal_(i+1)
        joy.header.stamp = ros::Time::now();
        ROS_INFO("Sending Joy msg %d to move_base",i+1);
        ac.sendGoal(joy);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Malish, the robot started joy action %d", i+1);
        else
            ROS_INFO("The robot failed joy action %d for some reason", i+1);
    }

}

void JoyCallback::sendInterrupt(const interface::ProcessInterface::Parameters &params) {  
}
