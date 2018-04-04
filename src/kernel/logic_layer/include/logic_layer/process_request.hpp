//
// Created by vladislav on 29.09.16.
//

#ifndef PROJECT_REQUEST_PARAM_HPP
#define PROJECT_REQUEST_PARAM_HPP

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <actionlib/client/simple_action_client.h>
#include <core_msgs/LLL_to_UserAction.h>

/**
 * @brief Структура данных, описывающая запрос к пользователю
 * @desc Содержит данные о запросе и умеет совершать данный запрос
 */
class ProcessRequest {
    static const int SERVER_TIMEOUT = 5;
    static const std::string SERVER_TIMEOUT_MESSAGE;

    typedef actionlib::SimpleActionClient<core_msgs::LLL_to_UserAction> Client;
    typedef std::shared_ptr<Client> ClientPtr;

    ClientPtr client;
    actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::PENDING;

public:
    typedef std::function<void (bool success, const std::string&)> callbackFunc;

    std::string text;
    std::string param;
    std::map<std::string, std::string> value_to_state;
    bool isVariable;

    int priority;

    void setPriority(int p);
    int getPriority();
    actionlib::SimpleClientGoalState getState();
    void send(const callbackFunc& callback = nullptr);
    core_msgs::LLL_to_UserResultConstPtr wait();
    void cancel();
};

typedef std::shared_ptr<ProcessRequest> ProcessRequestPtr;

#endif //PROJECT_REQUEST_PARAM_HPP
