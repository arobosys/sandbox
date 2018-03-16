//
// Created by vladislav on 04.08.16.
//

#ifndef PROJECT_ROSLINKSERVER_HPP
#define PROJECT_ROSLINKSERVER_HPP

#include <actionlib/server/simple_action_server.h>

#include <core_msgs/LLL_to_PLAction.h>
#include <atomic>

/**
 * @brief Сервер RosLink
 * @details Класс обеспечивает связь c LLL.
 *          События обрабатываются посредством регистрации соответствующих callback'ов
 */
class RosLinkServer {
    typedef core_msgs::ProcessData ProcessData;
    typedef actionlib::SimpleActionServer<core_msgs::LLL_to_PLAction> LLL_to_PL;
    typedef std::shared_ptr<LLL_to_PL> LLL_to_PLPtr;
    typedef std::function<void(std::atomic_bool &isInterrupted, const std::vector<int32_t> &ID)> KillCallback;
    typedef std::function<void(const std::vector<ProcessData> &goalData, LLL_to_PLPtr &result)> DataCallback;

    const std::string mName = "core_process_layer";

    ros::NodeHandle mNh;
    LLL_to_PLPtr mAs_LLL_to_PL;

    std::vector<ProcessData> lastGoalData;

    KillCallback killCallback;
    DataCallback dataCallback;

    /**
     * @brief Callback для принятия goal от LLL
     * @details Принимает и отправляет goal в dataCallback
     */
    void goalLLLCB();

    /**
     * @brief Callback для принятия статуса preempt от LLL
     * @details Убивает все процессы
     */
    void preemptLLLCB();

public:

    /**
     * @brief Регистрирует DataCallback
     * @param dataCb Функтор для обработки данных
     */
    void registerDataCallback(const DataCallback &dataCb);

    /**
     * @brief Регистрирует KillCallback
     * @param killCb Функтор для убийства процессов
     */
    void registerKillCallback(const KillCallback &killCb);

    void sendFeedback(const PLData & feedback);

    /**
     * @brief Конструктор сервера RosLink
     * @details Запускает ActionServer и Service для связи с LLL
     * @param nh NodeHandle данного узла ROS
     */
    RosLinkServer(const ros::NodeHandle &nh);
};

typedef std::shared_ptr<RosLinkServer> RosLinkServerPtr;

#endif //PROJECT_ROSLINKSERVER_HPP
