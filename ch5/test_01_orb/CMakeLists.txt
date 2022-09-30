cmake_minimum_required(VERSION 3.0)
project(ComputeORB)

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

SET(CMAKE_CXX_STANDARD 17)

#寻找opencv库
find_package(OpenCV REQUIRED)
#find_package(TBB REQUIRED)

#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} "/usr/local/include/eigen3" "/usr/local/include/sophus")
#include_directories(${TBB_INCLUDE_DIRS})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)

add_executable(computeORB computeORB.cpp)
add_executable(test test.cpp)

#链接OpenCV库 和tbb库
target_link_libraries(computeORB ${OpenCV_LIBRARIES} tbb)
target_link_libraries(test ${OpenCV_LIBRARIES})