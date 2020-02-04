cmake_minimum_required(VERSION 2.8.3)
project(libelas)
set(CMAKE_BUILD_TYPE Release)

# use sse3 instruction set
add_definitions(-msse3)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES elas
)

add_library(elas src/descriptor.cpp src/elas.cpp src/filter.cpp src/matrix.cpp src/triangle.cpp)
include_directories(include include/libelas ${catkin_INCLUDE_DIRS})

install(
  TARGETS elas
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
