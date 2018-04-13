#ifndef JOY_PROCESS_CLASS__
#define JOY_PROCESS_CLASS__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Joy.h"
#include <core_msgs/DataPL.h>
#include <process_interface/process_interface.hpp>

using interface::ProcessInterface;

struct Pointline
{
    int id;

    float x, y, w;

    int r, g, b;
};

class JoyProcess{

    void joyCommandCallback(const sensor_msgs::Joy command);

    ros::Subscriber joystik;
    ros::Publisher led_pub_;
    bool _gogogo, _restart;
    int _mode;
    int bindbuttons[10];
    bool goalisactiv;
    interface::ProcessInterface *rosLinkClientPtr = nullptr;
    void setpointcolor(Pointline p);
    void sendgoal(Pointline p);

public:
    typedef std::function<void()> FeedbackHandler;
    void  setPointByNum(std::string num);
    /**
     * Class constructor
     */
    JoyProcess(ros::NodeHandle &handle, int argc, char **argv);
    void SetFeedbackActive();
    int setmode(int mode);

    void SetUpProcessInterface(interface::ProcessInterface* const process){
        rosLinkClientPtr = process;
    }

private:
    FeedbackHandler feedbackHandler;
};

#endif //JOY_PROCESS_CLASS__