#include "process_interface/parent_process_interface.hpp"

namespace interface {

    void ParentProcessInterface::feedbackCB(const std::vector<core_msgs::DataPL> &fb) {
        ProcessInterface::Feedback feedback;
        feedback.feedback = fb;
        _processInterface.publishFeedback(feedback);
    }

    void ParentProcessInterface::goalHandler(const ProcessInterface::Parameters &goal) {
        if (!_goalPrehandler(goal)) {
            return;
        }
        auto childs = _processInterface.getAllProcessInfos();

        if (_firstGoal) {
            _firstGoal = false;
            for (auto &a : childs) {
                _processInterface.setChildEnabled(a.id, true);
            }
        }

        auto feedbackManager = std::make_shared<FeedbackManager>(
                std::bind(&ParentProcessInterface::feedbackCB, this, std::placeholders::_1));
        _processInterface.setManager(feedbackManager);
        _processInterface.startChilds();
    }

    const ProcessInterface &ParentProcessInterface::processInterface() const {
        return _processInterface;
    }

    ParentProcessInterface::ParentProcessInterface(int argc, char **argv, GoalPrehandler goalPrehandler)
            : _processInterface(argc, argv), _goalPrehandler(goalPrehandler) {
        _processInterface.setGoalHandler(std::bind(&ParentProcessInterface::goalHandler, this, std::placeholders::_1));
        _processInterface.listen();
    }

}

