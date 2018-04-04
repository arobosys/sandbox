#ifndef COMMAND_INTERPRETER_HPP
#define COMMAND_INTERPRETER_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "command_parser.hpp"
#include <core_msgs/CI_to_HLLAction.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Class for communication with user. It takes user's commands from input topic and converts it to CI_to_HLLGoal, then passing it to HLL
 */
class command_executor {
private:
    typedef actionlib::SimpleActionClient<core_msgs::CI_to_HLLAction> CI_to_HLLClient;

    /**
     * Wrapper for client waitForServer method
     * @param client client to wait on
     */
    template<typename T>
    void waitForServer(actionlib::SimpleActionClient<T> &client);

    /**
     * Wrapper for client waitForResult method
     * @param client client to wait on
     */
    template<typename T>
    void waitForResult(actionlib::SimpleActionClient<T> &client);

public:
    command_executor();

    /**
     * Callback for user commands from input topic
     * @param msg received message from topic
     */
    void inputCallback(const std_msgs::String::ConstPtr msg);
};

#endif // COMMAND_INTERPRETER_HPP
