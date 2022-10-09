cmake_minimum_required(VERSION 2.8)

project(test)

Find_Package(Eigen3 REQUIRED)
Find_Package(Sophus REQUIRED)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIR}  )

add_library(bal_common common.cc)

add_executable(test test.cpp)

target_link_libraries(test ${Sophus_LIBRARIES} "/usr/local/lib/libg2o.so" "/opt/ros/melodic/lib/libg2o_solver_csparse.so" bal_common)
