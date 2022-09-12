cmake_minimum_required( VERSION 2.8 )

project(tra)

find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS}  )

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(tra draw_trajectory.cpp )
add_executable(test test.cpp )
target_link_libraries(tra
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
        )

