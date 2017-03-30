project(SLAM_CPP)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   slam_02.cpp
   robot.cpp)

add_executable(SLAM_CPP ${sources})
