cmake_minimum_required(VERSION 2.4.6)
project(map_handler)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  localizer
  gvg_mapper
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES target_servo
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs nav_msgs localizer gvg_mapper
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_executable(map_handler src/map_handler.cpp)

target_link_libraries(map_handler
  ${catkin_LIBRARIES}
)
