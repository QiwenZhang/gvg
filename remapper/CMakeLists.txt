cmake_minimum_required(VERSION 2.4.6)
project(remapper)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES target_servo
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(remapper src/remapper.cpp)

target_link_libraries(remapper
  ${catkin_LIBRARIES}
)
