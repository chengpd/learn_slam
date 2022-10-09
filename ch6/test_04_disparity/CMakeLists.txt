cmake_minimum_required(VERSION 3.22)
project(test_03_zjf)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} "/usr/include/Eigen3")


add_executable(test_03_zjf
        direct_method.cpp)
target_link_libraries(test_03_zjf  ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES})