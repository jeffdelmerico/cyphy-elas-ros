cmake_minimum_required(VERSION 2.8.3)
project(libelas)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

catkin_simple()

# use sse3 instruction set
add_definitions(-msse3)

cs_add_library(elas
  libelas/src/descriptor.cpp
  libelas/src/elas.cpp
  libelas/src/filter.cpp
  libelas/src/matrix.cpp
  libelas/src/triangle.cpp)

#include_directories(include ${catkin_INCLUDE_DIRS})

# catkin_package(
#   INCLUDE_DIRS libelas/src
#   LIBRARIES elas
# )

cs_install()
cs_export()
