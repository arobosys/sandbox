#include "process_interface/process_interface.hpp"

namespace interface {

    ProcessInterface::Parameters ProcessInterface::parseParams(const ParametersVec& params) const {
        Parameters myParams;

        auto it = std::find_if(params.begin(), params.end(),
                     [this] (const Parameters& p) {
                         return p.ID_Proc == this->processId;
                     });

        if (it != params.end())
            myParams = *it;

        return myParams;
    }

    void ProcessInterface::goalCB() {
        auto goal = server->acceptNewGoal();

        processId = goal->ID;
        params = std::move(goal->params);
        for (auto& p : goal->processes) {
            childsInfo[p.id] = p;
        }

        if (goalHandler) goalHandler(parseParams(params));
    }

    void ProcessInterface::preemptCB() const {
        if (preemptHandler) preemptHandler();
    }

    void ProcessInterface::startChilds(const ChildProcessInfo::ProcessExitCallback& exitCallback,
                                       const FeedbackHandler& fbCallback) {
        for (const auto& c : childsInfo) {
            if (!childToEnabled[c.first]) continue;

            ChildProcessInfo info(c.second, exitCallback, params);
            ChildProcessPtr childProcess = std::make_shared<ChildProcess>(info);

            auto feedbackCB = [&, fbCallback] (const FeedbackConstPtr& fb) {
                auto managerCB = std::bind(&FeedbackManager::feedbackCB, feedbackManager.get(), std::placeholders::_1);
                if (fbCallback) fbCallback(fb);
                managerCB(fb);
            };

            auto resultCB = [&, feedbackCB] (auto, const ResultConstPtr& result) {
                auto feedback = boost::make_shared<Feedback>();
                feedback->feedback = result->result;
                feedbackCB(feedback);
            };

            childProcess->sendGoal(feedbackCB, resultCB); //hehe check by testing

            childProcesses[c.first] = childProcess;
            childToEnabled[c.first] = false;
        }
    }

    void ProcessInterface::listen() {
        server->start();
    }

    std::vector<ChildProcessPtr> ProcessInterface::getAllProcesses() {
        std::vector<ChildProcessPtr> res;
        for (const auto& i : childProcesses) {
            res.push_back(i.second);
        }
        return res;
    }

    std::vector<core_msgs::ChildProcess> ProcessInterface::getAllProcessInfos() {
        std::vector<core_msgs::ChildProcess> res;
        for (const auto& i : childsInfo) {
            res.push_back(i.second);
        }
        return res;
    }


    void ProcessInterface::imitateGoalRecv(const Parameters& params) {
        if (goalHandler) goalHandler(params);
    }

    void ProcessInterface::setSucceeded(const Result &result, const std::string &text) const {
        server->setSucceeded(result, text);
    }

    void ProcessInterface::setPreempted(const Result &result, const std::string &text) const {
        server->setPreempted(result, text);
    }

    void ProcessInterface::setAborted(const Result &result, const std::string &text) const {
        server->setAborted(result, text);
    }

    void ProcessInterface::publishFeedback(const Feedback &feedback) const {
        server->publishFeedback(feedback);
    }

    void ProcessInterface::setGoalHandler(GoalHandler goalHandler) {
        this->goalHandler = goalHandler;
    }

    void ProcessInterface::setPreemptHandler(PreemptHandler preemptHandler) {
        this->preemptHandler = preemptHandler;
    }

    int ProcessInterface::getId() const {
        return processId;
    }

    std::string ProcessInterface::getName() const {
        return name;
    }

    ProcessInterface::ParametersVec ProcessInterface::getAllParameters() const {
        return params;
    }

    ProcessInterface::Parameters ProcessInterface::getMyParameters() const {
        return parseParams(params);
    }

    core_msgs::ChildProcess ProcessInterface::getChildInfo(int id) {
        return childsInfo[id];
    }

    void ProcessInterface::setChildEnabled(int id, bool value) {
        childToEnabled[id] = value;
    }

    ChildProcessPtr ProcessInterface::getProcess(int id) {
        return childProcesses[id];
    }

    void ProcessInterface::setManager(const FeedbackManagerPtr& manager) {
        feedbackManager = manager;
    }

    ProcessInterface::ProcessInterface(int argc, char **argv,
                                 GoalHandler goalHandler, PreemptHandler preemptHandler)
            : goalHandler(goalHandler), preemptHandler(preemptHandler) {
        if (argc < 2) throw std::invalid_argument("You should pass name of process as argument");

        name = argv[1];

        server = std::make_unique<Server>(name, false);
        server->registerGoalCallback(boost::bind(&ProcessInterface::goalCB, this));
        server->registerPreemptCallback(boost::bind(&ProcessInterface::preemptCB, this));
        ROS_INFO("Started process server with name <%s>", name.c_str());
    }

}