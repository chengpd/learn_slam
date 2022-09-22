cmake_minimum_required(VERSION 3.0)

project(tra)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw    draw_trajectory.cpp)
target_link_libraries(draw     ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES})



















#include_directories(
# include
#"/usr/include/GL/gl.h")


#cmake_minimum_required(VERSION 3.0)

#project(DrawTrajectory)

#find_package(Pangolin REQUIRED)
#find_package(Sophus REQUIRED)

#include_directories(${Pangolin_INCLUDE_DIRS})
#include_directories(${Sophus_INCLUDE_DIRS})

#add_executable(draw_trajectory draw_trajectory.cpp)
#target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})cmake_minimum_required( VERSION 2.8 )

#project(tra)

#add_definitions(-std=c++11)

#find_package(Sophus REQUIRED)
#INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS} )

#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})

#add_executable(tra draw_trajectory.cpp )

#target_link_libraries(tra ${catkin_LIBRARIES} ${Sophus_LIBRARIES})
