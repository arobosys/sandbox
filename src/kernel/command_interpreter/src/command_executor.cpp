#include "command_executor.hpp"

const int SERVER_WAIT_TIMEOUT = 3;
const int RESULT_WAIT_TIMEOUT = 10;

template<typename T>
void command_executor::waitForServer(actionlib::SimpleActionClient<T> &client) {
    if (!(client.waitForServer(ros::Duration(SERVER_WAIT_TIMEOUT)))) {
        throw ros::Exception("Cannot connect to server");
    }
}

template<typename T>
void command_executor::waitForResult(actionlib::SimpleActionClient<T> &client) {
    if (!(client.waitForResult(ros::Duration(RESULT_WAIT_TIMEOUT)))) {
        throw ros::Exception("Result timeout exceeded");
    }
}

command_executor::command_executor(){}

void command_executor::inputCallback(const std_msgs::String::ConstPtr msg) {
    try {
        core_msgs::CI_to_HLLGoal goal = command_parser::parse(msg->data);
        ROS_INFO("CI successfuly parsed a command");

        CI_to_HLLClient client("core_hll", true);
        waitForServer<core_msgs::CI_to_HLLAction>(client);

        auto resultCB = [](auto, auto) {};
        auto activeCV = [] {};
        auto feedbackCB = [this] (const core_msgs::CI_to_HLLFeedbackConstPtr &feedback) {
            ROS_INFO("[CI feedback : stateHLL] %s", feedback->stateHLL.c_str());
            ROS_INFO("[CI feedback : stateLLL] %s", feedback->stateLLL.c_str());
        };

        client.sendGoal(goal, resultCB, activeCV, feedbackCB);
        client.waitForResult();
        ROS_INFO("[CI result : state] %s", client.getResult()->state.c_str());
    } catch (ParseError &exc) {
        ROS_WARN("%s", exc.what());
        ROS_WARN("%s", command_parser::usage.c_str());
    }
}
