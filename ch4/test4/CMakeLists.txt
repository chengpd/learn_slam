cmake_minimum_required(VERSION 3.2)

PROJECT(gaussnewton)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES("usr/include/eigen3")

ADD_EXECUTABLE(gaussnewton gaussnewton.cpp)

TARGET_LINK_LIBRARIES(gaussnewton ${OpenCV_LIBS})