#include <csignal>
#include "sqlite3_wrapper/environment.hpp"
#include "logic_layer/techmap_server.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "core_lll");
    ros::NodeHandle n;
    signal(SIGINT, [](int) { ros::shutdown(); quick_exit(0); });

    DatabasePtr db;
    try {
        db = std::make_shared<Database>(getAvisRoot() + "/database/avis.db");
    } catch (DatabaseRuntimeError& e) {
        ROS_ERROR("%s", e.what());
        return 1;
    }
    Mapping::setDatabase(db);

    LLL::TechmapServer srv("core_lll");
    ros::spin();

    return 0;
}