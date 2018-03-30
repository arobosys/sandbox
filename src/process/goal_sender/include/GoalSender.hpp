#ifndef __GOALSENDER_CLASS_
#define __GOALSENDER_CLASS_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
// #include <malish/JoyCMD.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalSender {
  //  ros::Publisher pub_lift;

    int num_goal;

    interface::ProcessInterface *rosLinkClientPtr = nullptr;

    std::map<std::string, std::string> parseTransforms(const std::map<std::string, std::string> &keyToValue);

    // void cmdCallback(const malish::JoyCMD &message);


public:

    void goalCallback(const interface::ProcessInterface::Parameters &params);

    void preemtCallback();

    void SetUpProcessInterface(interface::ProcessInterface* const process){
        rosLinkClientPtr = process;
    }

    /**
     * Class constructor
     */
    GoalSender(ros::NodeHandle &handle, int argc, char **argv);

};

#endif //__GOALSENDER_CLASS_
