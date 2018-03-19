#include <ros/ros.h>
#include <std_msgs/String.h>
#include <malish/JoyCMD.h>


class JoyProcess{

    void joyCommandCallback(const malish::JoyCMD &command);

    ros::Subscriber joystik;
    bool _gogogo, _restart;
  
public:
    /**
     * Class constructor
     */
    JoyProcess(ros::NodeHandle &handle, int argc, char **argv);
};

