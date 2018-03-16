cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs message_generation)
add_action_files(DIRECTORY action FILES CI_to_HLL.action HLL_to_LLL.action LLL_to_PL.action LLL_to_User.action to_Process.action)
add_message_files(DIRECTORY msg FILES KeyValue.msg LL_Params.msg ChildProcess.msg ProcessData.msg DataPL.msg LogMessage.msg)
add_service_files(DIRECTORY srv FILES ProcessList.srv)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()
