#include "command_parser.hpp"
#include "command_executor.hpp"

const std::string DEFAULT_INPUT_TOPIC_NAME = "CI_input";

const int MSG_SIZE = 1024;

int main(int argc, char **argv) {
    ros::init(argc, argv, "CI");
    ros::NodeHandle n;

    std::string input_topic_name = DEFAULT_INPUT_TOPIC_NAME;
    if (argc > 1) {
        input_topic_name = std::string(argv[1]);
    }

    command_executor cm;
    ros::Subscriber subs = n.subscribe(input_topic_name, MSG_SIZE, &command_executor::inputCallback, &cm);

    ros::spin();
    return 0;
}
