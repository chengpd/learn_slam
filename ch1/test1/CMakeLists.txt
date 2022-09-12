project(hello1)

add_library(hellolib hello.cpp)

add_executable(sayhello1 sayhello.cpp)

add_executable(sayhello hello_slam.cpp)

target_link_libraries(sayhello1 hellolib)
