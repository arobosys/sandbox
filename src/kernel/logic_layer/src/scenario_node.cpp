#include "logic_layer/scenario_node.hpp"

namespace HLL {

    ScenarioNode::ScenarioNode(int id, bool out, const std::string &outState)
            : Node(id, out, outState) {}

    int ScenarioNode::getTechMapId() {
        return id;
    }

    std::string ScenarioNode::execute() {
        TechmapClient aclient("core_lll");
        ROS_INFO("TechmapClient waiting for server");
        if (!aclient.waitForServer(ros::Duration(SERVER_TIMEOUT)))
            return SERVER_TIMEOUT_MESSAGE;

        core_msgs::HLL_to_LLLGoal goal;
        goal.ID = id;
        goal.params = this->params;
        ROS_INFO("TechmapClient sending goal (ID=%d)", goal.ID);

        auto resultCB = [](auto, auto) {};
        auto activeCV = [] {};

        auto feedbackCB = [this](const core_msgs::HLL_to_LLLFeedbackConstPtr &feedback) {
            fbCb(feedback->stateLLL);
        };

        aclient.sendGoal(goal, resultCB, activeCV, feedbackCB);
        ROS_INFO("TechmapClient waiting for result");
        aclient.waitForResult();

        auto result = aclient.getResult();
        ROS_INFO("TechmapClient tmap execution ret state: %s", result->state.c_str());
        return result->state;
    }

}