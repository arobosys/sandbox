
add_executable(safety_test safety_test.cpp)
target_link_libraries(safety_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(safety_test malish_generate_messages_cpp)
