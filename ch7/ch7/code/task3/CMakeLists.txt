CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(DirectBA)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_STANDARD 14)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

#SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
set(G2O_ROOT /usr/local/include/g2o)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

add_executable(directBA directBA.cpp)

target_link_libraries(directBA ${G2O_LIBS}  ${OpenCV_LIBS}  ${Pangolin_LIBRARIES})
