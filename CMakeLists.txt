project(SLAM_CPP)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   slam_01.cpp)

add_executable(SLAM_CPP ${sources})
