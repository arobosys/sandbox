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
        auto data = goalParams.key_value;

        typedef std::map<std::string, std::string> map_SS_T;
        map_SS_T keyToValue, result;

        for (const auto &a : data) {
            keyToValue[a.key] = a.value;
        }

        for(auto wp = keyToValue.begin(); wp != keyToValue.end(); ++wp) {
            if (wp->first == "WP") {
                ROS_INFO("received WP from PL.");
                joystik.setPointByNum(wp->second.c_str());
                break;
            }
        }

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