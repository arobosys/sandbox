#include <map>
#include <unordered_map>
#include "lift_process.hpp"
#include <string>
#include <math.h>
#include <malish/Lift.h>

LiftProcess::LiftProcess(ros::NodeHandle &handle, int argc, char **argv) {
    rosLinkClientPtr_ =
        std::make_shared<interface::ProcessInterface>(argc, argv,
                                                      std::bind(&LiftProcess::liftCallback, this, std::placeholders::_1),
                                                      std::bind(&LiftProcess::preemtCallback, this));

    lift_pub_ = handle.advertise<malish::Lift>("/lift", 10);

    rosLinkClientPtr_->listen();
}

/*void
LiftProcess::SetUpProcessInterface(interface::ProcessInterface* const process) {
    rosLinkClientPtr_ = process;
    // rosLinkClientPtr_ = std::make_shared<interface::ProcessInterface>(process);
} */

void
LiftProcess::liftCallback(const interface::ProcessInterface::Parameters &params)
{
    interface::ProcessInterface::Result alibResultOk, alibResultFail;
    interface::ProcessInterface::Feedback feedback;
    core_msgs::DataPL dataPL;
    core_msgs::KeyValue keyValue;
    map_SS_T keyToValue;

    // Get key-value from tbOption
    auto data = params.key_value;

    dataPL.command = "execute";
    dataPL.id = rosLinkClientPtr_->getId();
    dataPL.type = "feedback";

    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }

    if(keyToValue.find("lift_up") != keyToValue.end()) {
        // Lift up.
        lift_msg_.dio1 = true;
        lift_msg_.dio2 = false;
        lift_pub_.publish(lift_msg_);

        keyValue.key = "lifted";
        keyValue.value = "OK";
        dataPL.states.push_back(keyValue);
    }

    if(keyToValue.find("lift_down") != keyToValue.end()) {
        // Reset lifts.
        lift_msg_.dio1 = false;
        lift_msg_.dio2 = false;
        lift_pub_.publish(lift_msg_);

        // Lift down.
        lift_msg_.dio1 = true;
        lift_msg_.dio2 = false;
        lift_pub_.publish(lift_msg_);

        keyValue.key = "delifted";
        keyValue.value = "OK";
        dataPL.states.push_back(keyValue);
    }

    feedback.feedback.push_back(std::move(dataPL));
    rosLinkClientPtr_->publishFeedback(std::move(feedback));

    rosLinkClientPtr_->setSucceeded(alibResultOk, "OK");
    ROS_INFO("Sended lift ResultOk");
}

void
LiftProcess::preemtCallback() {
	// Response to stop goal.
}
