cmake_minimum_required(VERSION 3.2)

project(MPU6050)
# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

file(GLOB SRC_FILES src/*.cpp)
add_executable(MPU6050 ${SRC_FILES})
