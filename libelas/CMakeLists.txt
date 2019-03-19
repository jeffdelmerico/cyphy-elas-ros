cmake_minimum_required(VERSION 2.8.3)
project(libelas)
set(CMAKE_BUILD_TYPE Release)

# use sse3 instruction set
add_definitions(-msse3)

find_package(catkin REQUIRED)

add_library(elas src/descriptor.cpp src/elas.cpp src/filter.cpp src/matrix.cpp src/triangle.cpp)
include_directories(include include/libelas ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elas
)
