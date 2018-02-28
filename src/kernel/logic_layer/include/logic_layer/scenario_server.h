#ifndef MSG_RECEIVER_H
#define MSG_RECEIVER_H

#include <actionlib/server/simple_action_server.h>
#include <core_msgs/CI_to_HLLAction.h>
#include "mapping.hpp"

namespace HLL {

    /***
     * Server for communication with Comand Interpreter
     * @author Svet
     */
    class ScenarioServer {
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<core_msgs::CI_to_HLLAction> as_;
        std::string action_name_;

        void goalCB();

        void preemptCB();

    public:

        ScenarioServer() = default;

        /***
         * Class constructor
         * @param name name for server
         */
        ScenarioServer(std::string name);

        /***
         * Shutting down server
         */
        void shutdown();

        /***
         * Returns server name
         * @return name of server]
         */
        std::string getName();

        /***
         * Class destructor. Shutting down server
         */
        ~ScenarioServer();
    };

}

#endif // MSG_RECEIVER_H
