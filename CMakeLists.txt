cmake_minimum_required(VERSION 2.8.3)
project(rmip)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  )

catkin_package(
 INCLUDE_DIRS
  include
     LIBRARIES
  rmip
 CATKIN_DEPENDS
  roscpp
  geometry_msgs
  roslib
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/cubic_spline_interpolator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ${PROJECT_NAME} ${catkin_LIBRARIES})
