#include <actionlib/server/simple_action_server.h>
#include <core_msgs/LLL_to_UserAction.h>

class ConsoleNode{
	ros::NodeHandle n;
	actionlib::SimpleActionServer<core_msgs::LLL_to_UserAction> server;

public:

	void executeCB(const core_msgs::LLL_to_UserGoalConstPtr &goal) {
		core_msgs::LLL_to_UserResult result;
	    std::cout << "[Console : goal text] " << goal->text << std::endl;

	    std::getline(std::cin, result.answer);
	    server.setSucceeded(result);
	    ROS_INFO("[Console] Got user's answer!");
	}

	ConsoleNode(std::string name): server(n, name, boost::bind(&ConsoleNode::executeCB, this, _1), false){
		server.start();
	}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "request_endpoint");

    ConsoleNode cn(ros::this_node::getName());

    ros::spin();
    return 0;
}
