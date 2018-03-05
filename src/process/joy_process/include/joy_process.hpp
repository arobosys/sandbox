#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <actionlib/client/simple_action_client.h>
#include <malish/Stop.h>

typedef actionlib::SimpleActionClient<malish::Stop> JoyClient;

class JoySender {
    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr = nullptr;

    void joyCallback(const interface::ProcessInterface::Parameters &params);
    void sendInterrupt(const std::map<std::string, std::string> &keyToValue);
  
public:
    /**
     * Class constructor
     */
    JoySender(ros::NodeHandle &handle, int argc, char **argv);
};

