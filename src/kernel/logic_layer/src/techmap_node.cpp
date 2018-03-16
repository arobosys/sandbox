//
// Created by vladislav on 13.09.16.
//

#include "logic_layer/techmap_node.hpp"

namespace LLL {

    const std::string TechMapNode::COMMAND_START = "start";
    const std::string TechMapNode::COMMAND_STOP = "stop";
    const std::string TechMapNode::COMMAND_RESTART = "restart";
    const std::string TechMapNode::OK = "OK";
    const std::string TechMapNode::FAIL = "FAIL";

    TechMapNode::TechMapNode(int nodeId, bool out, const std::string& outState,
                             const ProcessVec& data, const ProcessRequestPtr& request)
            : Node(nodeId, out, outState), procData(data), requestParam(request) {
        for (const auto& pd : procData) {
            taskToStatus[pd.ID] = false;
        }
    }

    std::string TechMapNode::execute() {
        aclient = std::make_unique<ProcListExecClient>("core_process_layer");
        ROS_INFO("TaskListExecClient waiting for server");
        if (!aclient->waitForServer(ros::Duration(SERVER_TIMEOUT)))
            return SERVER_TIMEOUT_MESSAGE;

        for (auto& pd : procData) {
            pd.params = parseParameters(pd.ID, params);
            pd.processes = Mapping::get_child_processes_by_id(pd.ID);
        }

        if (requestParam) {
            requestParam->send();
            auto result = requestParam->wait();
            bool success = requestParam->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;

            if (success) {
                for (auto& pd : procData) {
                    for (auto& p : pd.params) {
                        for (auto& kv : p.key_value) {
                            if (kv.key == requestParam->param)
                                kv.value = result->answer;
                        }
                    }
                }
            } else {
                throw GraphUnexhaustedRuntimeError("Request to user failed");
            }
        }

        auto resultCB = [](auto, auto) {};
        auto activeCV = [] {};

        auto feedbackCB = [&] (const core_msgs::LLL_to_PLFeedbackConstPtr &feedback) {
            //ROS_INFO("Got feedback: %s", feedback->feedback[0].result.c_str());
            if (handleData(feedback->feedback)) {
                isGoalActive = false;
            }
        };

        core_msgs::LLL_to_PLGoal goal;
        goal.data = procData;
        aclient->sendGoal(goal, resultCB, activeCV, feedbackCB);
        isGoalActive = true;
        while (!aclient->waitForResult(ros::Duration(0.5)) && isGoalActive);
        aclient->cancelGoal();

        if (aclient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !handleData(aclient->getResult()->result))
            throw GraphUnexhaustedRuntimeError("Can't find correct state");
        ROS_INFO("TechMapExecClient tmap execution ret state: %s", outState.c_str());

        return outState;
    }

    bool TechMapNode::handleData(const std::vector<core_msgs::DataPL>& data) {
        for (const auto &d : data) {
            if (d.command == "execute") {
                ProcessGroup::group_t kv;
                for (const auto &s : d.states) {
                    kv.emplace_back(s.key, s.value);
                }

                auto pgVec = Mapping::get_process_group(d.id);
                decltype(pgVec) matched;
                for (const auto& pg : pgVec) {
                    if (pg.match(kv))
                        matched.push_back(pg);
                }

                if (!matched.empty()) {
                    auto pg = *std::max_element(matched.begin(), matched.end(),
                                                [](const auto &a, const auto &b) {
                                                    return a.priority < b.priority;
                                                });

                    if (pg.isRequest) {
                        if (requestState != nullptr) {
                            return false; //TODO multiple requests
                        }

                        requestState = std::make_shared<ProcessRequest>(Mapping::get_request(pg.requestId));
                        requestState->setPriority(pg.priority);

                        isInterrupted = false;
                        requestThread = std::make_unique<std::thread>([&] {
                            requestState->send([&](bool success, const std::string &answer) {
                                if (success) {
                                    outState = requestState->value_to_state[answer];
                                    if (outState != "") {
                                        if (outState != "{null}") isGoalActive = false;
                                        isInterrupted = true;
                                    }
                                } else {
                                    ROS_ERROR("Request answer is bad");
                                }
                            });

                            ros::Rate rate(5);
                            while (ros::ok() && !isInterrupted) {
                                ros::spinOnce();
                                rate.sleep();
                            }

                            if (isInterrupted) {
                                requestState->cancel();
                            }
                            requestState = nullptr;
                        });

                        return false;
                    } else if (requestState && pg.priority > requestState->priority || !requestState) {
                        outState = Mapping::get_state(pg.stateId);
                        return true;
                    }
                }
            } else if (d.command == COMMAND_START || d.command == COMMAND_STOP || d.command == COMMAND_RESTART) {
                if (d.result == OK) {
                    taskToStatus[d.id] = true;
                }
                if (d.result == FAIL) {
                    outState = FAIL;
                    return true;
                }


            } else {
                throw std::runtime_error("Unknown command");
            }
        }

        bool ready = true;
        for (const auto& i : taskToStatus) {
            ready &= i.second;
        }

        if (ready) {
            outState = OK;
            return true;
        }

        return false;
    }

}