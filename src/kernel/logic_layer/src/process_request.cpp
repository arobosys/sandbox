#include "logic_layer/process_request.hpp"

void ProcessRequest::send(const callbackFunc& callback) {
    client = std::make_shared<Client>("request_endpoint");
    if (!client->waitForServer(ros::Duration(SERVER_TIMEOUT)))
        throw std::runtime_error("Console node server unavailable");

    core_msgs::LLL_to_UserGoal goal;
    goal.text = this->text;

    auto resultCB = [&] (const actionlib::SimpleClientGoalState& state,
                        const core_msgs::LLL_to_UserResultConstPtr result) {
        this->state = state;
        if (callback) {
            callback(state == actionlib::SimpleClientGoalState::SUCCEEDED, result->answer);
        }
    };

    client->sendGoal(goal, resultCB);
}

core_msgs::LLL_to_UserResultConstPtr ProcessRequest::wait() {
    client->waitForResult();
    return client->getResult();
}

void ProcessRequest::cancel() {
    client->cancelGoal();
}

int ProcessRequest::getPriority() {
    return priority;
}

void ProcessRequest::setPriority(int p) {
    priority = p;
}

actionlib::SimpleClientGoalState ProcessRequest::getState() {
    return state;
}