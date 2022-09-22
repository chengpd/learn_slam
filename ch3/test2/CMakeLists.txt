cmake_minimum_required(VERSION 3.0)

project(error)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(error error.cpp)
target_link_libraries(error   ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES})

