//
// Created by vladislav on 05.08.16.
//

#ifndef PROJECT_ROSLINKCLIENT_HPP
#define PROJECT_ROSLINKCLIENT_HPP

#include <string>
#include <stdexcept>
#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <core_msgs/to_ProcessAction.h>
#include "feedback_manager.hpp"

namespace interface {

/**
 * @brief Клиент RosLink
 * @details Класс обеспечивает связь с родительским процессом либо с менеджером процессов.
 */
    class ProcessInterface {
        typedef core_msgs::to_ProcessAction Action;
        typedef actionlib::SimpleActionServer<Action> Server;
        typedef std::unique_ptr<Server> ServerPtr;

    public:
        using Goal = Server::Goal;
        using Result = Server::Result;
        using Feedback = Server::Feedback;
        using GoalConstPtr = Server::GoalConstPtr;
        using FeedbackConstPtr = Server::FeedbackConstPtr;
        using ResultConstPtr = Server::ResultConstPtr;

        typedef core_msgs::LL_Params Parameters;
        typedef std::vector<core_msgs::LL_Params> ParametersVec;
        typedef std::function<void(const Parameters&)> GoalHandler;
        typedef std::function<void()> PreemptHandler;
        typedef std::function<void(const FeedbackConstPtr &feedback)> FeedbackHandler;

    private:

        ServerPtr server;
        GoalHandler goalHandler;
        PreemptHandler preemptHandler;

        int processId;
        std::string name;
        ParametersVec params;
        std::map<int, core_msgs::ChildProcess> childsInfo;
        std::map<int, bool> childToEnabled;

        FeedbackManagerPtr feedbackManager;
        std::map<int, ChildProcessPtr> childProcesses;

        /**
         * @brief Обработчик входящего goal. Переадресует на GoalHandler.
         */
        void goalCB();

        /**
         * @brief Обработчик статуса preempt. Переадресует на PreemptHandler.
         */
        void preemptCB() const;

        Parameters parseParams(const ParametersVec& params) const;

    public:
        void imitateGoalRecv(const Parameters& params);

        /**
         * @brief Устанавливает успешный результат
         * @param result Данные для родительского процесса или менеджера процессов
         * @param text Текстовое сообщение
         */
        void setSucceeded(const Result &result = Result(), const std::string &text = std::string()) const;

        /**
         * @brief Устанавливает результат Preempted
         * @param result Данные для родительского процесса или менеджера процессов
         * @param text Текстовое сообщение
         */
        void setPreempted(const Result &result = Result(), const std::string &text = std::string()) const;

        /**
         * @brief Устанавливает результат Aborted
         * @param result Данные для родительского процесса или менеджера процессов
         * @param text Текстовое сообщение
         */
        void setAborted(const Result &result = Result(), const std::string &text = std::string()) const;

        /**
         * @brief Отправляет feedback родительскому процессу или менеджеру процессов
         * @param feedback Данные для родительского процесса или менеджера процессов
         */
        void publishFeedback(const Feedback &feedback) const;

        void setGoalHandler(GoalHandler goalHandler);

        void setPreemptHandler(PreemptHandler preemptHandler);

        int getId() const;

        std::string getName() const;

        ParametersVec getAllParameters() const;

        Parameters getMyParameters() const;

        core_msgs::ChildProcess getChildInfo(int id);

        void setChildEnabled(int id, bool value);

        ChildProcessPtr getProcess(int id);

        void setManager(const FeedbackManagerPtr& manager);

        std::vector<ChildProcessPtr> getAllProcesses();

        std::vector<core_msgs::ChildProcess> getAllProcessInfos();

        void startChilds(const ChildProcessInfo::ProcessExitCallback& exitCallback = nullptr,
                         const FeedbackHandler& fbCallback = nullptr);

        void listen();

        /**
         * @brief Конструктор клиента RosLink
         * @details Запускает ActionServer с именем из аргументов командной строки.
         * @param argc, argv Параметры командной строки. Первый аргумент должен содержать имя для соединения по ActionLib
         * @param goalHandler Обработчик входящего goal от родительского процесса или менеджера процессов
         * @param preemptHandler Обработчик статуса preempt от родительского процесса или менеджера процессов
         */
        ProcessInterface(int argc, char *argv[], GoalHandler goalHandler = nullptr,
                      PreemptHandler preemptHandler = nullptr);
    };

}

#endif //PROJECT_ROSLINKCLIENT_HPP
