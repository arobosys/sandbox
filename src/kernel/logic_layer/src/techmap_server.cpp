#include "logic_layer/techmap_server.hpp"

namespace LLL {

    TechmapServer::TechmapServer(std::string name)
            : as(nh, name, false), action_name(name) {
        as.registerGoalCallback(boost::bind(&LLL::TechmapServer::goalCB, this));
        as.start();
        ROS_INFO("TechmapServer started");
    }

    void TechmapServer::goalCB() {
        auto goal = as.acceptNewGoal();
        ROS_INFO("TechmapServer executing tmap (ID=%d)", goal->ID);
        core_msgs::HLL_to_LLLResult result;
        ScenarioExecutorPtr techmap;
        try {
            ROS_INFO("ID = %d", goal->ID);
            techmap = Mapping::get_tech_map_by_id(goal->ID);
        } catch (const std::exception &exc) {
            result.state = "error";
            as.setAborted(result);
            ROS_ERROR("Caught exception in get_tech_map_by_id in goalCB() with what(): %s", exc.what());
            return;
        }

        techmap->registerFeedbackCallback(
                [this](const std::string &fbHLL, const std::string &fbLLL) {
                    core_msgs::HLL_to_LLLFeedback feedback;
                    feedback.stateLLL = fbLLL;
                    as.publishFeedback(feedback);
                });

        result.state = techmap->execute();
        as.setSucceeded(result);
    }

}