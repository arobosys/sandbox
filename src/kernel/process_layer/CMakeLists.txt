cmake_minimum_required(VERSION 2.8.3)
project(process_layer)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
        actionlib
        roscpp
        core_msgs
        tiny_process
        )

catkin_package(CATKIN_DEPENDS core_msgs tiny_process)
catkin_add_gtest(pl_test test/test.cpp)

include_directories(
        ${catkin_INCLUDE_DIRS} include src
)

add_library(thread_pool STATIC src/threadpool.cpp)

add_executable(process_layer src/main.cpp src/proccontrol.cpp src/roslinkserver.cpp)

target_link_libraries(process_layer ${catkin_LIBRARIES} thread_pool)
target_link_libraries(pl_test ${catkin_LIBRARIES} thread_pool_lib ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_dependencies(process_layer ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})