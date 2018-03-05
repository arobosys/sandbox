#include "joy_process.hpp"
typedef actionlib::SimpleActionClient<sensor_msgs::Joy> JoyClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_sender_node");
    ros::NodeHandle n;
    ROS_INFO("joy_sender_node initializing...");
    JoySender  joySender(n,argc,argv);
    ROS_INFO("joy_sender_node finished sending...");
    ros::spin();
    return 0;
}
