cmake_minimum_required(VERSION 3.13)
project(convolution_filter)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

find_package(PNG REQUIRED)

add_library(reader read.hpp read.cpp)
target_link_libraries(reader PNG::PNG)

add_library(writer write.hpp write.cpp)
target_link_libraries(writer PNG::PNG)

add_library(algorithm algorithm.hpp algorithm.cpp)
target_link_libraries(algorithm writer PNG::PNG)

add_executable(convolution_filter main.cpp)
target_link_libraries(convolution_filter reader writer algorithm PNG::PNG)

configure_file(input.png "${CMAKE_CURRENT_BINARY_DIR}/input.png" COPYONLY)