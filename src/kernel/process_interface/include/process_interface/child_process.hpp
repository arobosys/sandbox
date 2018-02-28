//
// Created by vladislav on 09.08.16.
//

#ifndef PROJECT_CHILDPROCESS_HPP
#define PROJECT_CHILDPROCESS_HPP

#include <string>
#include <sstream>
#include <atomic>
#include <actionlib/client/simple_action_client.h>
#include <core_msgs/to_ProcessAction.h>
#include "process.hpp"

namespace interface {

/**
 * @brief Перечисление кодов ошибок
 */
    enum ErrorCode {
        NoError = 0,
        IncorrectInputData = 1,
        ProcessNotFound = 2, //есть в БД, но путь к нему неверный
        ProcessNotExist = 3, //даже в БД отсутствует
        ProcessNotStarted = 4,
        ProcessFinished = 5
    };

/**
 * @brief Перечисление приоритетов процессов
 */
    enum Priority {
        HIGH,
        MIDDLE,
        LOW
    };

/**
 * @brief Дочерний процесс
 * @details Класс позволяет создавать дочерний процесс, подключаться к нему через ActionLib и отправлять ему задачи
 * @tparam T Сообщение ActionLib
 */

/**
 * Common structure for child process info
 * proc_id - id of process
 * name - name of process
 * path - path to process binary
 * exitCallback - callback to notify if process has died
 * feedbackCallback - callback to send feedback to parent
 * args - arguments to processes
 */
    struct ChildProcessInfo {
        typedef std::function<void(int procId, int exitCode)> ProcessExitCallback;

        int proc_id;
        std::string name;
        std::string path;
        ProcessExitCallback exitCallback;
        std::vector<std::string> args;
        std::vector<core_msgs::LL_Params> params;
        std::vector<core_msgs::ChildProcess> processes;

        ChildProcessInfo();
        ChildProcessInfo(const core_msgs::ChildProcess &cp, const ProcessExitCallback &callback,
                         const std::vector<core_msgs::LL_Params> &params,
                         const std::vector<std::string> &args = {});
    };

    class ChildProcess {
    public:
        typedef core_msgs::to_ProcessAction Action;
        typedef typename Action::_action_goal_type::_goal_type Goal;
        typedef typename Action::_action_feedback_type::_feedback_type Feedback;
        typedef typename Action::_action_result_type::_result_type Result;
        typedef typename actionlib::SimpleActionClient<Action>::SimpleDoneCallback SimpleDoneCallback;
        typedef typename actionlib::SimpleActionClient<Action>::SimpleFeedbackCallback SimpleFeedbackCallback;
        typedef typename actionlib::SimpleActionClient<Action>::SimpleActiveCallback SimpleActiveCallback;

    private:
        typedef actionlib::SimpleActionClient<Action> Client;
        typedef std::unique_ptr<Client> ClientPtr;
        typedef std::shared_ptr<Process> ProcessPtr;

        static const int PROCCESS_START_TIMEOUT_MSEC;
        static const int PROCCESS_CONNECT_TIMEOUT_SEC;

        ClientPtr client;
        ProcessPtr process;

        ChildProcessInfo info;

        std::atomic<ErrorCode> errorCode;

        /**
         * @brief Преобразует абсолютный путь в имя процесса и путь к папке этого процесса
         * @param path[in] Абсолютный путь
         * @param name[out] Имя файла процесса
         * @param dir[out] Папка процесса
         * @param args[in] Аргументы запуска процесса (добавляются в имя процесса)
         */
        static void pathToNameAndDir(const std::string &path, std::string &name, std::string &dir,
                                     std::vector<std::string> args = std::vector<std::string>());

        static void expandEnvVars(std::string &unexpanded);

        static std::string expandPath(const std::string &unexpanded);

    public:
        /**
         * @brief Возвращает код текущей ошибки
         * @return Код ошибки
         */
        ErrorCode getErrorCode();

        /**
         * @brief Получает текстовое значение кода ошибки
         * @return Строка, содержащая текстовое значение кода ошибки
         */
        std::string errorString();

        /**
         * @brief Отмена goal.
         */
        void cancelGoal();

        /**
        * @brief Убивает процесс
        * @param force Если true - отправляет процессу SIGTERM, иначе - SIGINT.
        */
        void kill(bool force = false);

        /**
        * @brief Ожидает завершения процесса
        * @return Возвращает код выхода процесса
        */
        int waitForExit();

        /**
         * @brief Возвращает статус текущего goal
         * @return Cтатус текущего goal
         */
        actionlib::SimpleClientGoalState getState();

        /**
         * @brief it's wrapper for SimpleActionClient<ActionSpec>::waitForResult(...).
         * @param timeout timeout to wait. if it's ros::Duration(0,0), then wait for infinite time
         * @return if SimpleActionClient finished in DONE state
         */
        bool waitForResult(const ros::Duration &timeout);

        /**
        * @brief Отправляет goal процессу.
        * @param goal Данные для процесса.
        * @param done_cb Обработчик результата.
        * @param feedback_cb Обработчик feedback.
        * @param active_cb Обработчик принятия goal.
        */
        void sendGoal(SimpleFeedbackCallback feed_cb = SimpleFeedbackCallback(),
                             SimpleDoneCallback done_cb = SimpleDoneCallback(),
                             SimpleActiveCallback active_cb = SimpleActiveCallback());

        /**
         * Getter for process id
         */
        int getProcId();

        /**
         * Setter for process params
         * @param params new params to apply
         */
        void setParams(const std::vector<core_msgs::LL_Params> &params);

        /**
        * @brief Конструктор дочернего процесса
        * @details Запускает дочерний процесс, присоединяется к нему через ActionLib
        * @param name Имя для связи через ActionLib
        * @param path Абсолютный путь к исполняемому файлу процесса
        * @param args Аргументы командной строки. Первым аргументом всегда автоматически подставляется name.
        */
        ChildProcess(const ChildProcessInfo &info);

        /**
        * @brief Деструктор. Убивает процесс.
        */
        ~ChildProcess();
    };

    typedef std::shared_ptr<ChildProcess> ChildProcessPtr;

}

#endif //PROJECT_CHILDPROCESS_HPP
