cmake_minimum_required(VERSION 2.8.3)
project(command_interpreter)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
if (COMPILER_SUPPORTS_CXX1Y)
    set(CMAKE_CXX_FLAGS "-std=c++1y")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++1y support. Please use a different C++ compiler.")
endif ()

find_package(catkin REQUIRED COMPONENTS roscpp core_msgs actionlib)

catkin_package(
        CATKIN_DEPENDS core_msgs
)

catkin_add_gtest(interpreter_test test/test.cpp)

add_library(parser_lib STATIC
        src/command_parser.cpp
        src/command_executor.cpp)

include_directories(include src ${catkin_INCLUDE_DIRS})

add_executable(command_interpreter src/CI_node.cpp)
target_link_libraries(command_interpreter parser_lib ${catkin_LIBRARIES})
target_link_libraries(interpreter_test parser_lib)

add_dependencies(parser_lib ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})