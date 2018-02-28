cmake_minimum_required(VERSION 2.8.3)
project(process_interface)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
        actionlib
        roscpp
        core_msgs
        tiny_process
        )

catkin_package(
        CATKIN_DEPENDS core_msgs tiny_process
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_library(${PROJECT_NAME} SHARED
        src/feedback_manager.cpp
        src/child_process.cpp
        src/process_interface.cpp
        src/parent_process_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
