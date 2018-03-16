#ifndef CHILD_MANAGER
#define CHILD_MANAGER

#include "child_process.hpp"
#include <map>
#include <vector>
#include <atomic>

namespace interface {

/**
 * Class for collecting feedbacks from multiple children
 */
    class FeedbackManager {
        typedef core_msgs::to_ProcessFeedbackConstPtr FeedbackConstPtr;
        typedef std::function<void(const std::vector<core_msgs::DataPL>&)> ParentFeedback;

        static const int COLLECT_FREQUENCY;

        std::vector<FeedbackConstPtr> feedbacks;
        std::mutex mutex;

        std::atomic_bool isCollecting;

        ParentFeedback feedbackToParent;

        void collectFB();

    public:
        /**
         * Class constructor
         * @param foo function to call on feedback collected
         * @param children list of children to watch
         */
        FeedbackManager(const ParentFeedback &foo);

        /**
         * Default function to store received feedback in common vector
         * @param feedback
         */
        void feedbackCB(const FeedbackConstPtr &feedback);

        /**
         * Stops collecting feedbacks
         */
        void stopAll();

        ~FeedbackManager();
    };

    typedef std::shared_ptr<FeedbackManager> FeedbackManagerPtr;

}

#endif //CHILD_MANAGER
