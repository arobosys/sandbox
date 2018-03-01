
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>


class GoalSender {
    /*
    const std::string GOAL1 = "2D_goal1";
    const std::string GOAL2 = "2D_goal2";
    const std::string GOAL3 = "2D_goal3";
    const std::string GOAL4 = "2D_goal4";
    const uint32_t QUEUE_SIZE = 100;
    const int TF_TIMEOUT_SEC = 5;
     */

    const std::string NUM_GOAL = "num_goal";
    const std::string GOAL1 = "2d_goal1";
    const std::string GOAL2 = "2d_goal2";
    const std::string GOAL3 = "2d_goal3";
    const std::string GOAL4 = "2d_goal4";

    int num_goal=2;

    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr = nullptr;

    //void startCallback(const std_msgs::String::ConstPtr &msg);

    void goalCallback(const interface::ProcessInterface::Parameters &params);

    void preemtCallback();

    void parseTransforms(const std::map<std::string, std::string> &keyToValue);

public:
    /**
     * Class constructor
     */
    GoalSender(ros::NodeHandle &handle, int argc, char **argv);

};

