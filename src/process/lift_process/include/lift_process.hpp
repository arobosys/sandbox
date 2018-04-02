#ifndef __LIFT_PROCESS_CLASS_
#define __LIFT_PROCESS_CLASS_

#include <list>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <actionlib/client/simple_action_client.h>
#include <malish/Lift.h>

typedef std::map<std::string, std::string> map_SS_T;

class LiftProcess {
public:;
    LiftProcess(ros::NodeHandle &handle, int argc, char **argv);

    void liftCallback(const interface::ProcessInterface::Parameters &params);

    void preemtCallback();

protected:;
    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr_ = nullptr;
    // Publisher lifters' messages
    ros::Publisher lift_pub_;
    malish::Lift lift_msg_;
};

#endif //__LIFT_PROCESS_CLASS_
