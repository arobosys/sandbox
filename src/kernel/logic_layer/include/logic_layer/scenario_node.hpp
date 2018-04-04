#ifndef TECH_MAP_NODE_H_
#define TECH_MAP_NODE_H_

#include "node.hpp"
#include <actionlib/client/simple_action_client.h>
#include <core_msgs/HLL_to_LLLAction.h>
#include <string>
#include <iostream>

namespace HLL {

    /**
     * @brief regular HLL node. stores techMapId, then sends it through actionlib to server. waits for result
     */
    class ScenarioNode : public Node {

        typedef actionlib::SimpleActionClient<core_msgs::HLL_to_LLLAction> TechmapClient;

    public:
        /**
         * Class constructor
         * @param techMapId id of techmap to execute
         * @param nodeId id of this node
         * @param fn
         * @param out is node terminal
         */
        ScenarioNode(int id, bool out = false, const std::string& outState = std::string());

        /**
         * Getter to techMapId
         * @return id of techmap to execute
         */
        int getTechMapId();

        virtual std::string execute() override;
    };

}; // namespace HLL

#endif // TECH_MAP_NODE_H_
