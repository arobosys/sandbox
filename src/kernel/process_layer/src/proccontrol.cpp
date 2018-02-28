#include <boost/xpressive/xpressive.hpp>
#include <threadpool.hpp>
#include "proccontrol.hpp"

const std::string START_STR = "start";
const std::string TERMINATE_STR = "terminate";
const std::string EXECUTE_STR = "execute";

Commands ProcControl::stringCmdToInt(const std::string &command) {
    if (command == START_STR) {
        return Commands::START;
    } else if (command == TERMINATE_STR) {
        return Commands::TERMINATE;
    } else if (command == EXECUTE_STR) {
        return Commands::EXECUTE;
    }
}

ProcControl::ProcControl() {
    feedbackFuture = threadPool->enqueue(std::bind(&ProcControl::spinCollecting, this, std::placeholders::_1));
}

ProcControl::~ProcControl() {
    threadPool->cancelTask(feedbackFuture);
}

void ProcControl::registerFeedbackCallback(const ProcControl::FeedbackCallback &foo) {
    feedbackCallback = foo;
}

void ProcControl::spinCollecting(std::atomic_bool &isInterrupted) {
    ros::Rate rate(10);
    while (!isInterrupted && ros::ok()) {
        auto collected = collectFeedbacks();
        if (feedbackCallback && !collected.empty()) {
            PLData feedback;
            feedback.type = 0;
            feedback.data = collected;
            feedbackCallback(feedback);
        }
        rate.sleep();
    }
}

// TODO: may be we should return ExtendedFuture to work with it outside the function
void ProcControl::newDataFromLLL(const std::vector<ProcessData> &goalData,
                                 std::shared_ptr<actionlib::SimpleActionServer<core_msgs::LLL_to_PLAction>> &result) {
    m_errorCode = NoError;
    this->callback_server = result;
    // TODO:check if the process exists in DB
    std::vector<ExtendedFuture<void> > currentFutures;
    for (const ProcessData &gd : goalData) {
        switch (stringCmdToInt(gd.command)) {
            case Commands::START : {
                auto future = threadPool->enqueue(
                        std::bind(&ProcControl::startProcess, this, std::placeholders::_1, gd));
                currentFutures.push_back(future);
                break;
            }
            case Commands::TERMINATE : {
                std::vector<int> ids;
                ids.push_back(gd.ID);
                auto future = threadPool->enqueue(
                        std::bind(&ProcControl::killProcess, this, std::placeholders::_1, ids, true));
                currentFutures.push_back(future);
                break;
            }
            case Commands::EXECUTE : {
                Goal goal;
                goal.ID = gd.ID;
                goal.params = gd.params;
                goal.processes = gd.processes;

                auto future = threadPool->enqueue(
                        std::bind(&ProcControl::sendGoal, this, std::placeholders::_1, gd.ID, goal));
                currentFutures.push_back(future);
                break;
            }
            default: {
                core_msgs::LLL_to_PLResult res;
                m_errorCode = IncorrectInputData;
                callback_server->setAborted(res, "Unknown command from LLL");
            }
        }
    }

//    // wait for all tasks to finish
//    for (auto& a : currentFutures) {
//        a.future.get();
//    }
    //threadPool->cancelTask(future);
    // collect all data that remains and send it as part of result
//    PLData taskListResult;
//    taskListResult.type = 1;
//    taskListResult.data = collectFeedbacks();
//    if (feedbackCallback)
//        feedbackCallback(taskListResult);
}

void ProcControl::expandEnvVars(std::string &unexpanded) {
    using namespace boost::xpressive;
    unexpanded =
        regex_replace(unexpanded, sregex::compile("\\$\\{([a-zA-Z_][a-zA-Z0-9_]*)\\}"), [](const smatch &match) {
            auto envvar = match[1].str();
            if (auto envval = ::getenv(envvar.c_str())) {
                return std::string(envval);
            }
            return envvar;
        });
}

std::string ProcControl::expandPath(const std::string &unexpanded) {
    std::string expanded = unexpanded;
    expandEnvVars(expanded);
    return expanded;
}

void ProcControl::startProcess(std::atomic_bool &isInterrupted, const ProcessData &pd) {
    if (isInterrupted) {
        m_errorCode = ProcessNotStarted;
        return;
    }

    std::string name = pd.name;

    std::string exec, dir;
    pathToNameAndDir(expandPath(pd.filepath), exec, dir, {name});

    ProcessPtr proc = std::make_shared<Process>(exec, dir,
                                                [&](int exitCode) {
                                                    if (exitCode != 0) {
                                                        errorCodes[pd.ID] = ProcessFinished;
                                                    }
                                                });

    if (!isInterrupted && errorCodes[pd.ID] != ErrorCode::NoError) {
        core_msgs::LLL_to_PLFeedback result;
        core_msgs::DataPL data;
        data.id = pd.ID;
        data.command = pd.command;
        switch (pd.priority) {
            case LOW: {// just send error to LLL
                data.type = "result";
                result.feedback.push_back(data);
                m_errorCode = ProcessNotStarted;
                safe_push(feedbackQueue, result);
                break;
            }
            case MIDDLE: {// send msg about error and try to restart
                core_msgs::LLL_to_PLFeedback feedback;
                data.type = "feedback";
                feedback.feedback.push_back(data);
                m_errorCode = ProcessNotStarted;
                safe_push(feedbackQueue, feedback);

                proc = std::make_shared<Process>(exec, dir,
                                                 [&](int exitCode) {
                                                     if (exitCode != 0) {
                                                         this->errorCodes[pd.ID] = ProcessFinished;
                                                     }

                                                 });
                if (!isInterrupted && this->errorCodes[pd.ID] != ErrorCode::NoError) {
                    data.type = "result";
                    data.result = "FAIL";
                    result.feedback.push_back(data);
                    m_errorCode = ProcessNotStarted;
                    safe_push(feedbackQueue, result);

                    std::vector<int32_t> process_ids;
                    for (auto t : mAs_PL_to_Process) {
                        process_ids.push_back(t.first);
                    }
                    killProcess(isInterrupted, process_ids);
                }
                m_errorCode = NoError;
                break;
            }
            case HIGH: { // send error to LLL and cancel all current tasks
                data.type = "result";
                data.result = "FAIL";
                result.feedback.push_back(data);
                safe_push(feedbackQueue, result);
                m_errorCode = ProcessNotStarted;

                std::vector<int32_t> process_ids;
                for (auto& t : mAs_PL_to_Process) {
                    process_ids.push_back(t.first);
                }
                killProcess(isInterrupted, process_ids);
                break;
            }
            default:
                throw ros::Exception("Priority of process is unknown");
        }
    }

    m_Processes[pd.ID] = proc;
    processToActiveGoal[pd.ID] = false; //TODO RACE!!!

    PL_to_ProcessPtr pl_to_process = std::make_shared<PL_to_Process>(name);
    if (!isInterrupted && !pl_to_process->waitForServer(ros::Duration(PROCCESS_CONNECT_TIMEOUT))) {
        throw ros::Exception("Action server of process is unreachable");
    }

    core_msgs::LLL_to_PLFeedback result;
    core_msgs::DataPL data;
    data.type="result";
    data.result="OK";
    data.command = pd.command;
    data.id = pd.ID;
    result.feedback.push_back(data);
    safe_push(feedbackQueue, result);

    mAs_PL_to_Process[pd.ID] = pl_to_process;
}

void ProcControl::feedbackCB(const core_msgs::to_ProcessFeedbackConstPtr &feedback) {
    core_msgs::LLL_to_PLFeedback currentFB;
    currentFB.feedback.insert(currentFB.feedback.end(), feedback->feedback.begin(), feedback->feedback.end());
    safe_push(feedbackQueue, currentFB);
}

std::vector<core_msgs::DataPL> ProcControl::collectFeedbacks() {
    std::unique_lock<std::mutex> lock(queue_mutex);

    std::vector<core_msgs::DataPL> collectedFeedbacks;
    size_t queueSize = feedbackQueue.size();
    for (size_t i = 0; i < queueSize; i++) {
        auto& fbs = feedbackQueue.front().feedback;
        collectedFeedbacks.insert(collectedFeedbacks.end(), fbs.begin(), fbs.end());
        feedbackQueue.pop();
    }
    return collectedFeedbacks;
}

void ProcControl::resultCB(const actionlib::SimpleClientGoalState &state,
                           const core_msgs::to_ProcessResultConstPtr &result) {
    core_msgs::LLL_to_PLFeedback res;
    res.feedback.insert(res.feedback.end(), result->result.begin(), result->result.end());
    safe_push(feedbackQueue, res);
    for (auto& f : result->result) {
        processToActiveGoal[f.id] = false;
    }
}

void ProcControl::sendGoal(std::atomic_bool &isInterrupted, int ID, const Goal &goal) {
    if (isInterrupted) {
        return;
    }
    auto a = mAs_PL_to_Process[ID];
    a->sendGoal(goal,
                boost::bind(&ProcControl::resultCB, this, _1, _2),
                PL_to_Process::SimpleActiveCallback(),
                boost::bind(&ProcControl::feedbackCB, this, _1));

    processToActiveGoal[ID] = true;

    while (!a->waitForResult(ros::Duration(0.5)) && processToActiveGoal[ID]);
}

inline void ProcControl::cancelGoal(int ID) {
    auto& proc = mAs_PL_to_Process[ID];
    if (proc && proc->isServerConnected()) {
        proc->cancelGoal();
    }
}

void ProcControl::killProcess(std::atomic_bool &isInterrupted, const std::vector<int32_t> &ID, bool forceKill) {
    for (size_t i = 0; i < ID.size() && !isInterrupted; i++) {
        this->cancelGoal(ID[i]);
        processToActiveGoal[ID[i]] = false;
        m_Processes[ID[i]]->kill(forceKill);

        core_msgs::LLL_to_PLFeedback result;
        core_msgs::DataPL data;
        data.type="result";
        data.result="OK";
        data.command = "terminate";
        data.id = ID[i];
        result.feedback.push_back(data);
        safe_push(feedbackQueue, result);
    }
}

std::string ProcControl::getPathProcess(int32_t ID) {
    char *path = std::getenv("AVIS_ROOT");
    if (path == nullptr) {
        throw std::runtime_error("Try to set up AVIS_ROOT variable!");
    }
    return std::string(path) + std::string("kernel/test_bin_proc/process_template");
}

std::string ProcControl::errorString(ErrorCode errorCode) {
    switch (errorCode) {
        case NoError:
            return "No errors";
        case ProcessNotExist:
            return "Process doesn't exist in DB";
        case ProcessNotFound:
            return "Process is not found";
        case ProcessNotStarted:
            return "Process is not started";
        case IncorrectInputData:
            return "Incorrect input data from LLL";
        case ProcessFinished:
            return "Process unexpectedly finished";
        default:
            return "Unknown errorCode";
    }
}

inline ErrorCode ProcControl::getErrorCode() {
    return m_errorCode;
}

void ProcControl::pathToNameAndDir(const std::string &path, std::string &name, std::string &dir,
                                   std::vector<std::string> args) {
    std::stringstream command;
    size_t idx = path.find_last_of('/');

    command << '.';
    if (idx == std::string::npos) {
        command << path;
        dir = std::string();
    } else {
        command << path.substr(idx);
        dir = path.substr(0, idx);
    }
    command << ' ';

    copy(args.begin(), args.end(), std::ostream_iterator<std::string>(command, " "));
    name = command.str();
}
