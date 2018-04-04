#include "joy_process_class.hpp"

#include <process_interface/process_interface.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_sender_node");
    ros::NodeHandle n;
    ROS_INFO("joy_sender_node initializing...");
    JoyProcess joystik(n,argc, argv);

    using interface::ProcessInterface;
    ProcessInterface::Result alibResultOk, alibResultFail;

    /**
     * Сейчас работа с Goals осуществляется следующим образом.
     *  % если приходит { "optionName": "start" }, то алгоритм стартует и отправляет feedback -- результат
     *    обработки каждого нового кадра алгоритмом. У rosLinkClient .setSucceeded никогда не вызывается,
     *    но может вызваться .setAborted в случае возникновения исключения в алгоритме или обработчике входных опций.
     *  % если приходит { "optionName": "stop" }, то алгоритму издается указ остановиться, на этот goal отправляется
     *    .setSuccedeed в случае успешного завершения алгоритма, .setAborted в случае возникновения исключения в
     *    процессе остановки алгоритма или обработчике входных опций.
     */

    ProcessInterface process(argc, argv);

    auto goalHandler = [&](const ProcessInterface::Parameters &goalParams) {
        ROS_INFO("received options from parent.");
        for (const auto& opt: goalParams.key_value) {
            if (opt.key == "command") {
                joystik.setmode(std::stoi(opt.value));
            }
        }
    };

    auto preemptHandler = [&] {};

    process.setGoalHandler(goalHandler);
    process.setPreemptHandler(preemptHandler);

    process.listen();

    auto joyFeedback = [&]() {
        // has state changed:
        ProcessInterface::Feedback feedback;
        core_msgs::DataPL dataPL;
        dataPL.command = "execute";
        dataPL.id = process.getId();
        dataPL.type = "feedback";
        core_msgs::KeyValue keyValue;
        keyValue.key = "setmod";
        keyValue.value = "OK";
        ROS_INFO("Status: %s", keyValue.value.c_str());

        dataPL.states.push_back(keyValue);

        feedback.feedback.push_back(std::move(dataPL));
        process.publishFeedback(std::move(feedback));
    };

    joystik.SetFeedback(joyFeedback);

    ROS_INFO("joy_sender_node initialized");
    ros::spin();
    return EXIT_SUCCESS;
}
