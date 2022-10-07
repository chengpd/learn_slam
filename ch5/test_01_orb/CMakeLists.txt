cmake_minimum_required(VERSION 3.0)
project(ComputeORB)

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/cp)

add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBRARIES})
target_link_libraries(test Threads::Threads)