#include "process_interface/child_process.hpp"
#include <boost/xpressive/xpressive.hpp>

namespace interface {

    const int ChildProcess::PROCCESS_START_TIMEOUT_MSEC = 500;
    const int ChildProcess::PROCCESS_CONNECT_TIMEOUT_SEC = 10;

    ChildProcessInfo::ChildProcessInfo() : proc_id(0) {}

    ChildProcessInfo::ChildProcessInfo(const core_msgs::ChildProcess &cp, const ProcessExitCallback &callback,
                                       const std::vector<core_msgs::LL_Params> &params,
                                       const std::vector<std::string> &args)
            : proc_id(cp.id), name(cp.name), path(cp.filepath),
              exitCallback(callback), args(args) {
        // search for required params in list
        for (auto a : params) {
            if (!a.path.empty() && a.path[0] == proc_id) {
                a.path.erase(a.path.begin());
                this->params.push_back(a);
            }
        }
    }

    void ChildProcess::expandEnvVars(std::string &unexpanded) {
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

    std::string ChildProcess::expandPath(const std::string &unexpanded) {
        std::string expanded = unexpanded;
        expandEnvVars(expanded);
        return expanded;
    }

    void ChildProcess::pathToNameAndDir(const std::string &path, std::string &name, std::string &dir,
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

    std::string ChildProcess::errorString() {
        switch (errorCode) {
            case NoError:
                return "No errorCode";
            case ProcessNotExist:
                return "Not found process in DB";
            case ProcessNotFound:
                return "Not found process";
            case ProcessNotStarted:
                return "Process is not started";
            case IncorrectInputData:
                return "Incorrect input data from Action";
            case ProcessFinished:
                return "Process unexpectedly finished";
            default:
                return "Unknown errorCode";
        }
    }

    ErrorCode ChildProcess::getErrorCode() {
        return errorCode;
    }

    void ChildProcess::cancelGoal() {
        client->cancelGoal();
    }

    void ChildProcess::kill(bool force) {
        process->kill(force);
    }

    int ChildProcess::waitForExit() {
        return process->get_exit_status();
    }

    actionlib::SimpleClientGoalState ChildProcess::getState() {
        return client->getState();
    }

    bool ChildProcess::waitForResult(const ros::Duration &timeout) {
        return client->waitForResult(timeout);
    }

    void ChildProcess::sendGoal(SimpleFeedbackCallback feed_cb,
                         SimpleDoneCallback done_cb,
                         SimpleActiveCallback active_cb) {
        Goal goal;
        goal.ID = info.proc_id;
        goal.params = info.params;
        goal.processes = info.processes;

        client->sendGoal(goal, done_cb, active_cb, feed_cb);
    }

    int ChildProcess::getProcId() {
        return info.proc_id;
    }

    void ChildProcess::setParams(const std::vector<core_msgs::LL_Params> &params) {
        info.params = params;
    }

    ChildProcess::ChildProcess(const ChildProcessInfo &i) : errorCode(NoError) {
        info = i;
        info.args.push_back(info.name);

        std::string exec, dir;
        pathToNameAndDir(expandPath(info.path), exec, dir, info.args);

        process = std::make_shared<Process>(exec, dir,
                                            [this](int exitCode) {
                                                if (info.exitCallback) {
                                                    info.exitCallback(info.proc_id, exitCode);
                                                }
                                                if (exitCode != 0) {
                                                    this->errorCode = ProcessFinished;
                                                }
                                            });
        std::this_thread::sleep_for(std::chrono::milliseconds(PROCCESS_START_TIMEOUT_MSEC));

        if (errorCode) {
            throw ros::Exception(errorString());
        }

        ROS_INFO("Child process started: <%s>", i.name.c_str());

        client = std::make_unique<Client>(info.name);
        if (!client->waitForServer(ros::Duration(PROCCESS_CONNECT_TIMEOUT_SEC))) {
            throw std::runtime_error("Action server of child process is unreachable: " + info.name);
        }
        ROS_INFO("Child process connected: <%s>", i.name.c_str());
    }

    ChildProcess::~ChildProcess() {
        this->kill(true);
    }

}