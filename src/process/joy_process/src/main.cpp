#include "joy_process_class.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_sender_node");
    ros::NodeHandle n;
    ROS_INFO("joy_sender_node initializing...");
    JoyProcess  joySender(n,argc,argv);
    ROS_INFO("joy_sender_node initialized");
    ros::spin();
    return 0;
}
