cmake_minimum_required(VERSION 2.8.3)
project(libelas)

find_package(catkin REQUIRED)

# use sse3 instruction set
add_definitions(-msse3)

add_library(elas
  libelas/src/descriptor.cpp
  libelas/src/elas.cpp
  libelas/src/filter.cpp
  libelas/src/matrix.cpp
  libelas/src/triangle.cpp)

include_directories(libelas/src ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS libelas/src
  LIBRARIES elas
)
