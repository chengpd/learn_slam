cmake_minimum_required(VERSION 3.22)
project(test_01_lk)

find_package(OpenCV)
find_package(Threads REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRECTORIES} "/usr/include/Eigen3")


add_executable(test_01_lk  optical_flow.cpp)
target_link_libraries(test_01_lk ${OpenCV_LIBRARIES} Threads::Threads)