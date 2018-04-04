#ifndef PROCCONTROL_H
#define PROCCONTROL_H

#include <memory>
#include <map>
#include <process.hpp>
#include <queue>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <core_msgs/to_ProcessAction.h>
#include <core_msgs/LLL_to_PLAction.h>
#include <core_msgs/ProcessData.h>
#include "threadpool.hpp"

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
    HIGH,   //Остановка работы
    MIDDLE, //Перезапуск
    LOW     //Сообщить об ошибке
};

/**
 * @brief Enum for possible commands to apply
 */
enum Commands {
    START,
    EXECUTE,
    TERMINATE
};

/**
 * Structure for feedback/result message to LLL
 * type: 0 - feedback, 1 - result
 */
struct PLData {
    int type;
    std::vector<core_msgs::DataPL> data;
};

/**
 * @brief Менеджер процессов
 * @details Класс предназначен для запуска процессов, соединения с ними через ActionLib
 *          и обмена сообщениями. Также может следить за состоянием процессов и убивать
 *          их по надобности.
 */
class ProcControl {
    typedef std::shared_ptr<actionlib::SimpleActionServer<core_msgs::LLL_to_PLAction>> LLL_to_PLServer;
    typedef core_msgs::ProcessData ProcessData;
    typedef actionlib::SimpleActionClient<core_msgs::to_ProcessAction> PL_to_Process;
    typedef std::shared_ptr<PL_to_Process> PL_to_ProcessPtr;
    typedef std::shared_ptr<Process> ProcessPtr;
    typedef std::function<void(const PLData &)> FeedbackCallback;

    static const int PROCCESS_CONNECT_TIMEOUT = 5;
    size_t WORKERS_COUNT = 10;

    static const std::string START;
    static const std::string EXECUTE;
    static const std::string STOP;
    static const std::string RESTART;

    std::map<int32_t, PL_to_ProcessPtr> mAs_PL_to_Process;
    std::map<int32_t, ProcessPtr> m_Processes;
    std::map<int32_t, ErrorCode> errorCodes;
    std::map<int32_t, std::atomic_bool> processToActiveGoal;
    ErrorCode m_errorCode = NoError;
    LLL_to_PLServer callback_server;

    std::shared_ptr<ThreadPool> threadPool = std::make_shared<ThreadPool>(WORKERS_COUNT);
    std::queue<core_msgs::LLL_to_PLFeedback> feedbackQueue;
    std::function<void(const PLData &)> feedbackCallback;

    std::mutex queue_mutex;

    ExtendedFuture<void> feedbackFuture;

    template <class T>
    void safe_push(std::queue<T>& q, const T& value) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        q.push(value);
    }

    static void expandEnvVars(std::string &unexpanded);

    static std::string expandPath(const std::string &unexpanded);

    /**
     * @brief Получает путь к исполняемому файлу процесса по ID
     * @warning Это заглушка
     * @param ID ID процесса
     * @return Возвращает строку, содержащую абсолютный путь к процессу
     */
    std::string getPathProcess(int32_t ID);

    /**
     * @brief Преобразует абсолютный путь в имя процесса и путь к папке этого процесса
     * @param path[in] Абсолютный путь
     * @param name[out] Имя файла процесса
     * @param dir[out] Папка процесса
     * @param args[in] Аргументы запуска процесса (добавляются в имя процесса)
     */
    static void pathToNameAndDir(const std::string &path, std::string &name, std::string &dir,
                                 std::vector<std::string> args);

    /**
     * @brief Принимает feedback от процессов, отправляет информацию в LLL
     * @param feedback Данные от процесса
     */
    void feedbackCB(const core_msgs::to_ProcessFeedbackConstPtr &feedback);

    /**
     * @brief Принимает result от процессов, отправляет информацию в LLL
     * @param state Состояние goal
     * @param result Данные от процесса
     */
    void resultCB(const actionlib::SimpleClientGoalState &state,
                  const core_msgs::to_ProcessResultConstPtr &result);

    /** @brief Convert input command as integer for switch by enum
     * @param s input command
     * @return Enum value for input command
     */
    Commands stringCmdToInt(const std::string &s);

    /**
     * @brief Collects feedback and results for queue
     * @return Current received feedbacks and results from processes
     */
    std::vector<core_msgs::DataPL> collectFeedbacks();

    /**
     * Spins collecting of feedbacks
     */
    void spinCollecting(std::atomic_bool &isInterrupted);

public:
    typedef typename core_msgs::to_ProcessGoal Goal;

    /**
     * @brief Отправляет goal процессу с данным ID
     * @param ID ID процесса
     * @param goal Данные для процесса
     */
    void sendGoal(std::atomic_bool &isInterrupted, int ID, const Goal &goal);

    void registerFeedbackCallback(const FeedbackCallback &foo);

    /**
     * @brief Отменяет goal процессу с данным ID
     * @param ID ID процесса
     */
    inline void cancelGoal(int ID);

    /**
     * @brief Запускает процессы с данными ID, приоритетами, настройками и параметрами командной строки
     * @param pd ProcessData из LLL
     * @throw ros::Exception in case unknown command reseived or process server is unreachable
     */
    void startProcess(std::atomic_bool &isInterrupted, const ProcessData &pd);

    /**
     * @brief Обработка сообщения от LLL
     * @details Метод выполняет соответствующую команду
     * @param processData
     *        ID Массив ID процессов для запуска
     *        command Команда для процесса, берется из Enum'а
     *        priority Приоритеты соответствующих процессов
     *        options Параметры соответствующих процессов
     *        params Аргументы командной строки соответствующих процессов
     */
    void newDataFromLLL(const std::vector<ProcessData> &goalData,
                        LLL_to_PLServer &result);

    /**
     * @brief Убивает процессы с данными ID
     * @param ID Массив ID процессов
     */
    void killProcess(std::atomic_bool &isInterrupted, const std::vector<int32_t> &ID, bool forceKill = true);

    /**
     * @brief Возвращает код ошибки
     * @return Код ошибки
     */
    inline ErrorCode getErrorCode();

    /**
     * @brief Получает текстовое значение кода ошибки
     * @param errorCode Код ошибки
     * @return Строка, содержащая текстовое значение кода ошибки
     */
    std::string errorString(ErrorCode errorCode);

    ProcControl();
    ~ProcControl();
};

#endif //PROCCONTROL_H
