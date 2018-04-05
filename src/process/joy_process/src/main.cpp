#include "joy_process_class.hpp"
#include <process_interface/process_interface.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_sender_node");
    ros::NodeHandle n;
    ROS_INFO("joy_sender_node initializing...");

    using interface::ProcessInterface;

    JoyProcess joystik(n,argc, argv);

    ProcessInterface process(argc, argv);

    joystik.SetUpProcessInterface(&process);

    auto goalHandler = [&](const ProcessInterface::Parameters &goalParams) {
        ROS_INFO("received goal from PL.");
        joystik.SetFeedbackActive();
    };

    auto preemptHandler = [&] {};

    process.setGoalHandler(goalHandler);
    process.setPreemptHandler(preemptHandler);

    process.listen();

    ROS_INFO("joy_sender_node initialized");
    ros::spin();
    return EXIT_SUCCESS;
}