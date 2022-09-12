
cmake_minimum_required(VERSION 2.8)

project(hello)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/lib)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

MESSAGE(STATUS " ni shuo de dou dui!")

add_library(sayhellolib SHARED src/sayHello.cpp)

add_library(sayhellostaticlib  src/sayHello.cpp)

add_executable(main main.cpp)

target_link_libraries(main sayhellolib)


#install
set(CMAKE_INSTALL_PREFIX /usr/local)

install(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION include)

install(TARGETS sayhellolib sayhellostaticlib
LIBRARY  DESTINATION lib
ARCHIVE  DESTINATION lib
)
