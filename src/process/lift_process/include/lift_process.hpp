#ifndef __LIFT_PROCESS_CLASS_
#define __LIFT_PROCESS_CLASS_

#include <list>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <malish/Lift.h>

typedef std::map<std::string, std::string> map_SS_T;

/*struct Goal {
	double x;
	double y;
	/// Rotation.
	double theta;
};*/

class LiftProcess {
public:;
    LiftProcess(ros::NodeHandle &handle, int argc, char **argv);

    void liftCallback(const interface::ProcessInterface::Parameters &params);

    void preemtCallback();

    // void SetUpProcessInterface(interface::ProcessInterface* const process);

    // std::vector<Goal> parseTransforms(const std::map<std::string, std::string> & goal_map);
    // void move(std::vector<Goal> const& goals_to_move);

protected:;
    // std::vector<std::string> goal_list_;
    // int num_goal_;
    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr_ = nullptr;
    // Publisher lifters' messages
    ros::Publisher lift_pub_;
    malish::Lift lift_msg_;
};

#endif //__GOALSENDER_CLASS_
