cmake_minimum_required(VERSION 3.13)
project(convolution_filter)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)

add_library(io io.hpp io.cpp)
target_link_libraries(io PNG::PNG JPEG::JPEG)

add_library(algorithm algorithm.hpp algorithm.cpp)
target_link_libraries(algorithm io)

add_executable(convolution_filter main.cpp)
target_link_libraries(convolution_filter io algorithm)