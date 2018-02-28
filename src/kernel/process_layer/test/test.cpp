//
// Created by vladislav on 13.08.16.
//

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <roslinkserver.hpp>
#include <proccontrol.hpp>

static int cnt = 1;


void infiniteCircle(std::atomic_bool &flag) {
    while (!flag) {
        cnt++;
    }
}

TEST(ThreadPool, cancelTask) {
    ThreadPool pool(10);
    std::vector<ExtendedFuture<void>> futures;
    size_t tasksCount = 50;
    for (size_t i = 0; i < tasksCount; i++) {
        futures.push_back(pool.enqueue(&infiniteCircle));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (size_t i = 0; i < tasksCount; i++) {
        EXPECT_EQ(i, futures[i].id);
        pool.cancelTask(futures[i]);
    }
    int prev = cnt;
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(prev, cnt);
    }
}

// TEST(process_layer, testGoal) {
//     namespace ph = std::placeholders;
//     ros::NodeHandle n;
//     ProcControl procControl(n);
//     RosLinkServerPtr rosLinkServer = std::make_shared<RosLinkServer>(n);
//     rosLinkServer->registerDataCallback(
//             std::bind(&ProcControl::newDataFromLLL, &procControl, ph::_1)
//     );
//     rosLinkServer->registerKillCallback(std::bind(&ProcControl::killProcess, &procControl, ph::_1));

//     procControl.sendGoal(1, ProcControl::Goal());
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "test_node");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}