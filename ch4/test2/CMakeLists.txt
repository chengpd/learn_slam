cmake_minimum_required( VERSION 2.8)

project(fisheye)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(${OpenCV})

add_executable(fisheye fisheye.cpp)

target_link_libraries(fisheye ${OpenCV_LIBS})