cmake_minimum_required(VERSION 2.8.9)
project(ttmath)
add_library(ttmath INTERFACE)
target_include_directories(ttmath INTERFACE ./)

catkin_package(
)
