#ifndef TECH_MAP_EXEC_SERVER_H_
#define TECH_MAP_EXEC_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <core_msgs/HLL_to_LLLAction.h>
#include <iostream>
#include "mapping.hpp"

namespace LLL {

/**
 * @brief actionlib server which executes given tech. map id (this is goal) and returns execution result as actionlib result
 */
    class TechmapServer {
    protected:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<core_msgs::HLL_to_LLLAction> as;
        std::string action_name;
        core_msgs::HLL_to_LLLFeedback feedback;
        core_msgs::HLL_to_LLLResult result;
    public:
        TechmapServer(std::string name = "tech_map_exec_al_server");

        void goalCB();

    };

}; // namespace LLL

#endif // TECH_MAP_EXEC_SERVER_H_
