cmake_minimum_required(VERSION 3.0)

project(ICP)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

#查找功能包
include_directories( "/usr/include/eigen3")
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(ICP ICP.cpp)
add_executable(ICP_1 ICP_1.cpp)

#链接OpenCV库
target_link_libraries(ICP ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries(ICP_1 ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )
