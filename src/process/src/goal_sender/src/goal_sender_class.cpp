#include <map>
#include <unordered_map>
#include "GoalSender.hpp"

GoalSender::GoalSender(ros::NodeHandle &handle, int argc, char **argv) {

    //goalListener = handle.subscribe(INPUT_TOPIC_NAME, 1, &GoalSenser::startCallback, this);
//rosLinkClientPtr = std::make_shared<interface::ProcessInterface>(argc, argv);

    rosLinkClientPtr = std::make_shared<interface::ProcessInterface>(argc, argv,
                                                                     std::bind(&GoalSender::goalCallback, this,
                                                                               std::placeholders::_1),
                                                                     std::bind(&GoalSender::preemtCallback,
                                                                               this));

    ROS_INFO("goal_sender_callback  is set");
    rosLinkClientPtr->listen();
}


void GoalSender::goalCallback(const interface::ProcessInterface::Parameters &params) {
    ROS_INFO("goal_sender_node is working");
    /*
    auto data = params.key_value;
    std::map<std::string, std::string> keyToValue;
    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }
    double goal = std::stod(keyToValue[GOAL1]);
    ROS_INFO("Got goal with 2D_goal1 = %f", goal);
     */
}

void GoalSender::preemtCallback() {}
