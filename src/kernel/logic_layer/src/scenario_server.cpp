//
// Created by vladislav on 13.06.16.
//

#include "logic_layer/scenario_server.h"

namespace HLL {

    ScenarioServer::ScenarioServer(std::string name) : as_(nh_, name, false),
                                                       action_name_(name) {
        as_.registerGoalCallback(boost::bind(&ScenarioServer::goalCB, this));
        as_.start();
        ROS_INFO("scenario_executor_server started!");
    }

    void ScenarioServer::goalCB() {
        auto goal = as_.acceptNewGoal();
        core_msgs::CI_to_HLLResult result_;

        ROS_INFO("Got a goal: %s %s", goal->command.c_str(), goal->alias.c_str());
        ScenarioExecutorPtr scenario;
        try {
            ROS_INFO("Alias = %s", goal->alias.c_str());
            scenario = Mapping::get_scenario_by_name(goal->alias);
        } catch (const std::exception &exc) {
            result_.state = "error";
            as_.setAborted(result_);
            ROS_ERROR("Caught exception in get_scenario_by_id in goalCB() with what(): %s", exc.what());
            return;
        }

        ROS_INFO("Got scenario from DB");

        //Replace parameters
        auto goalParams = goal->params;
        for (auto it = scenario->parametersBegin(); it != scenario->parametersEnd(); ++it) {
            auto &proc = *it;
            for (auto &p : proc.key_value) {
                auto itProc = std::find_if(goalParams.begin(), goalParams.end(),
                                           [&proc](const auto &pr) {
                                               return proc.ID_Proc == pr.ID_Proc && proc.ID_TL == pr.ID_TL;
                                           });
                if (itProc == goalParams.end()) continue;

                auto itParam = std::find_if(itProc->key_value.begin(), itProc->key_value.end(),
                                            [&p](const auto &par) {
                                                return p.key == par.key;
                                            });

                if (itParam != itProc->key_value.end()) {
                    p.value = itParam->value;
                    itProc->key_value.erase(itParam);
                    if (itProc->key_value.empty())
                        goalParams.erase(itProc);
                }
            }
        }

        if (!goalParams.empty()) {
            result_.state = "Wrong parameters";
            ROS_ERROR("Unused parameters detected");
            as_.setAborted(result_);
            return;
        }

        scenario->registerFeedbackCallback(
                [this](const std::string &fbHLL, const std::string &fbLLL) {
                    core_msgs::CI_to_HLLFeedback feedback;
                    feedback.stateHLL = fbHLL;
                    feedback.stateLLL = fbLLL;
                    as_.publishFeedback(feedback);
                });

        std::string result;
        try {
            result = scenario->execute();
        } catch (const GraphUnexhaustedRuntimeError &exc) {
            result = "Error";
            ROS_ERROR("GraphUnexhaustedRuntimeError caught in scenario->execute()");
        }

        result_.state = result;
        as_.setSucceeded(result_);
        ROS_INFO("Execution finished with state %s", result.c_str());
    }

    void ScenarioServer::preemptCB() {
        ROS_INFO("goal is preempted");
        as_.setPreempted();
    }

    void ScenarioServer::shutdown() {
        ROS_INFO("%s", "Shutting down server");
        as_.shutdown();
    }

    std::string ScenarioServer::getName() {
        return action_name_;
    }

    ScenarioServer::~ScenarioServer() {
        as_.shutdown();
    }

}