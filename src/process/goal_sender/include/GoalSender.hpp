
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <malish/Lift.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalSender {
  //  ros::Publisher pub_lift;

    const std::string NUM_GOAL = "num_goal";
    const std::string GOAL1 = "2d_goal1";
    const std::string GOAL2 = "2d_goal2";
    const std::string GOAL3 = "2d_goal3";
    const std::string GOAL4 = "2d_goal4";
    const std::string GOAL5 = "2d_goal5";
    const std::string GOAL6 = "2d_goal6";
    const std::string GOAL7 = "2d_goal7";

    int num_goal;


    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr = nullptr;

    void goalCallback(const interface::ProcessInterface::Parameters &params);

    void preemtCallback();

    void parseTransforms(const std::map<std::string, std::string> &keyToValue);


public:
    /**
     * Class constructor
     */
    GoalSender(ros::NodeHandle &handle, int argc, char **argv);

};

