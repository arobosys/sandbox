#include <ros/ros.h>
#include <std_msgs/String.h>
#include <malish/JoyCMD.h>


class JoyProcess{

    void joyCommandCallback(const malish::JoyCMD &command);

    ros::Subscriber joystik;
    bool _gogogo, _restart;
    int _mode;


  
public:
    typedef std::function<void()> FeedbackHandler;

    /**
     * Class constructor
     */
    JoyProcess(ros::NodeHandle &handle, int argc, char **argv);
    void SetFeedback(FeedbackHandler feedbackHandler);
    int setmode(int mode);

private:
    FeedbackHandler feedbackHandler;
};

