cmake_minimum_required(VERSION 3.2)

PROJECT(disparity)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
   
ADD_EXECUTABLE(disparity disparity.cpp)

target_link_libraries(disparity ${OpenCV_LIBRARIES}  ${Pangolin_LIBRARIES})
