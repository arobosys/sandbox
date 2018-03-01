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

template<typename T>
std::vector<T> splitByDelimiter(const std::string &s, char delim);

template<>
std::vector<double> splitByDelimiter(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<double> result;
    while (std::getline(ss, item, delim)) {
        result.push_back(std::stod(item));
    }
    return result;
}

void GoalSender::parseTransforms(const std::map<std::string, std::string> &keyToValue) {

    std::string goal1 = keyToValue.at(GOAL1);
    std::string goal2 = keyToValue.at(GOAL2);
    std::string goal3 = keyToValue.at(GOAL3);
    std::string goal4 = keyToValue.at(GOAL4);

    std::vector<double> goal = splitByDelimiter<double>(goal1, ';');
    ROS_INFO("The 2d_goal1: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goal = splitByDelimiter<double>(goal2, ';');
    ROS_INFO("The 2d_goal1: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goal = splitByDelimiter<double>(goal3, ';');
    ROS_INFO("The 2d_goal1: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);
    goal = splitByDelimiter<double>(goal4, ';');
    ROS_INFO("The 2d_goal1: x=%f, y=%f, theta=%f",goal[0], goal[1], goal[2]);

}

void GoalSender::goalCallback(const interface::ProcessInterface::Parameters &params) {
    ROS_INFO("goal_sender_node is working");
    auto data = params.key_value;
    std::map<std::string, std::string> keyToValue;
    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }
    //delta = std::stod(keyToValue[DELTA]);
    num_goal = std::stod(keyToValue[NUM_GOAL]);
    ROS_INFO("The number of goal = %d", num_goal);
    parseTransforms(keyToValue);
    //publishTrajectory();
    //ROS_INFO("Got goal with delta = %f", delta);
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
