cmake_minimum_required(VERSION 2.8.3)
project(tiny_process)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(include)
add_library(${PROJECT_NAME} STATIC
        src/process.cpp
        )
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})