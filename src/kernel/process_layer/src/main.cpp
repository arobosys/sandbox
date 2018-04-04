#include "proccontrol.hpp"
#include "roslinkserver.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "core_process_layer");
    ros::NodeHandle n;
    signal(SIGINT, [](int) { ros::shutdown(); quick_exit(0); });

    namespace ph = std::placeholders;
    ProcControl procControl;
    RosLinkServerPtr rosLinkServer = std::make_shared<RosLinkServer>(n);
    rosLinkServer->registerDataCallback(
            std::bind(&ProcControl::newDataFromLLL, &procControl, ph::_1, ph::_2)
    );

    const bool forceKill = true;
    rosLinkServer->registerKillCallback(std::bind(&ProcControl::killProcess, &procControl, ph::_1, ph::_2, forceKill));
    procControl.registerFeedbackCallback(std::bind(&RosLinkServer::sendFeedback, rosLinkServer.get(), ph::_1));

    ros::spin();

    return 0;
}
