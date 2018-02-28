//
// Created by vladislav on 04.08.16.
//

#include <proccontrol.hpp>
#include "roslinkserver.hpp"

RosLinkServer::RosLinkServer(const ros::NodeHandle &nh)
        : mNh(nh), dataCallback(nullptr), killCallback(nullptr) {
    mAs_LLL_to_PL = std::make_shared<LLL_to_PL>(nh, mName, false);

    mAs_LLL_to_PL->registerGoalCallback(boost::bind(&RosLinkServer::goalLLLCB, this));
    mAs_LLL_to_PL->registerPreemptCallback(boost::bind(&RosLinkServer::preemptLLLCB, this));
    mAs_LLL_to_PL->start();
    ROS_INFO("RosLinkServer started");
}

void RosLinkServer::goalLLLCB() {
    auto goal = mAs_LLL_to_PL->acceptNewGoal();

    lastGoalData = goal->data;

    if (dataCallback)
        dataCallback(goal->data, mAs_LLL_to_PL);
}

void RosLinkServer::preemptLLLCB() {
//    std::vector<int32_t> ID(lastGoalData.size());
//    for (const auto &i : lastGoalData) {
//        ID.push_back(i.ID);
//    }
//
//    std::atomic_bool flag;
//    flag = false;
//    if (killCallback) killCallback(std::ref(flag), ID);
}

void RosLinkServer::registerDataCallback(const RosLinkServer::DataCallback &dataCb) {
    dataCallback = dataCb;
}

void RosLinkServer::registerKillCallback(const RosLinkServer::KillCallback &killCb) {
    killCallback = killCb;
}

void RosLinkServer::sendFeedback(const PLData &feedback) {
    LLL_to_PL::Feedback fb;
    fb.feedback.insert(fb.feedback.end(), feedback.data.begin(), feedback.data.end());
    mAs_LLL_to_PL->publishFeedback(fb);
}
