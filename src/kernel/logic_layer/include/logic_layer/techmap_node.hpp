#ifndef PROC_LIST_NODE_H_
#define PROC_LIST_NODE_H_

#include <actionlib/client/simple_action_client.h>
#include <core_msgs/LLL_to_PLAction.h>
#include <thread>
#include <atomic>
#include "process_request.hpp"
#include "mapping.hpp"

namespace LLL {

/**
 * @brief regular LLL node
 */
    class TechMapNode : public Node {
        typedef actionlib::SimpleActionClient<core_msgs::LLL_to_PLAction> ProcListExecClient;
        typedef std::unique_ptr<ProcListExecClient> ProcListExecClientPtr;
        typedef std::vector<core_msgs::ProcessData> ProcessVec;

        static const std::string COMMAND_START;
        static const std::string COMMAND_STOP;
        static const std::string COMMAND_RESTART;
        static const std::string OK;
        static const std::string FAIL;

        ProcListExecClientPtr aclient;

        std::unique_ptr<std::thread> requestThread;
        std::atomic_bool isInterrupted;

        ProcessRequestPtr requestState, requestParam;
        ProcessVec procData;
        std::map<int, bool> taskToStatus;

        std::atomic_bool isGoalActive;

        std::string outState;

        bool handleData(const std::vector<core_msgs::DataPL>& data);

    public:
        TechMapNode(int nodeId, bool out, const std::string& outState,
                    const ProcessVec& data, const ProcessRequestPtr& request = nullptr);

        virtual std::string execute() override;
    };

}; // namespace LLL

#endif //PROC_LIST_NODE_H_
