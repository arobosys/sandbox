#include "process_interface/feedback_manager.hpp"

namespace interface {

    const int FeedbackManager::COLLECT_FREQUENCY = 4;

    void FeedbackManager::collectFB() {
        ros::Rate rate(COLLECT_FREQUENCY);
        while (isCollecting) {
            {
                std::unique_lock<std::mutex> lock(mutex);
                if (!feedbacks.empty()) {
                    std::vector<core_msgs::DataPL> feedbackData;

                    for (auto &f : feedbacks) {
                        feedbackData.insert(feedbackData.end(), f->feedback.begin(), f->feedback.end());
                    }

                    feedbacks.clear();
                    feedbackToParent(feedbackData);
                }
            }
            rate.sleep();
        }
    }

    FeedbackManager::FeedbackManager(const ParentFeedback &foo) {
        feedbackToParent = foo;

        isCollecting = true;
        std::thread collector(std::bind(&FeedbackManager::collectFB, this));
        collector.detach();
    }

    FeedbackManager::~FeedbackManager() {
        stopAll();
    }

    void FeedbackManager::feedbackCB(const FeedbackConstPtr &feedback) {
        std::unique_lock<std::mutex> lock(mutex);
        feedbacks.push_back(feedback);
    }

    void FeedbackManager::stopAll() {
        isCollecting = false;
    }

}