cmake_minimum_required(VERSION 2.8.3)
project(logic_layer)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS roscpp actionlib core_msgs)
catkin_package(
        CATKIN_DEPENDS core_msgs
        INCLUDE_DIRS include
        LIBRARIES sqlite3_wrapper)

include_directories(include src ${catkin_INCLUDE_DIRS})

add_library(sqlite3_wrapper SHARED
        src/database.cpp
        src/result_set.cpp
        src/environment.cpp)

add_library(scenario_executor SHARED
        src/graph.cpp
        src/scenario_node.cpp
        src/techmap_server.cpp
        src/scenario_server.cpp
        src/techmap_node.cpp
        src/mapping.cpp
        src/process_group.cpp
        src/node.cpp
        src/process_request.cpp)

target_link_libraries(sqlite3_wrapper sqlite3)
target_link_libraries(scenario_executor ${catkin_LIBRARIES} sqlite3_wrapper)
add_dependencies(scenario_executor ${catkin_EXPORTED_TARGETS})

add_executable(scenario_execserver src/rosnode_HLL.cpp)
add_executable(techmap_execserver src/rosnode_LLL.cpp)
target_link_libraries(scenario_execserver scenario_executor)
target_link_libraries(techmap_execserver scenario_executor)

install(TARGETS scenario_execserver techmap_execserver scenario_executor
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
        include/sqlite3_wrapper/database.hpp
        include/sqlite3_wrapper/result_set.hpp
        include/sqlite3_wrapper/environment.hpp
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/sqlite3_wrapper
        )
