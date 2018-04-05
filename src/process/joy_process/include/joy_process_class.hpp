#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Joy.h"
#include <core_msgs/DataPL.h>
#include <process_interface/process_interface.hpp>

//#include <malish/JoyCMD.h>

using interface::ProcessInterface;

class JoyProcess{

    struct Pointline
    {

        int id;

        float x;
        float y;
        float w;

        int r;
        int g;
        int b;

    };


    void joyCommandCallback(const sensor_msgs::Joy command);

    ros::Subscriber joystik;
    bool _gogogo, _restart;
    int _mode;
    int bindbuttons[10];
    bool goalisactiv;
    interface::ProcessInterface *rosLinkClientPtr = nullptr;

public:
    typedef std::function<void()> FeedbackHandler;

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

