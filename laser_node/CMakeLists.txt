cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(laser_node)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  visualization_msgs 
)

add_message_files(
DIRECTORY msg
FILES
  Corners.msg  
  Obstacle.msg  
  Obstacles.msg
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
generate_messages(DEPENDENCIES 
  std_msgs 
  sensor_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES basic_geometry
CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  visualization_msgs 
) 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  include
)
#SET ( CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -g" )

#common commands for building c++ executables and libraries
add_library(basic_geometry src/BasicGeometry.cpp)
add_dependencies(basic_geometry ${PROJECT_NAME}_generate_messages_cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(laserNode src/laserNode.cpp)
add_executable(laserUtilsServer src/main.cpp src/LaserUtils.cpp src/BasicGeometry.cpp)
add_executable(laser_utils_viz  src/viz.cpp)
#rosbuild_add_executable(test_wall_following  src/tests/test_wall_following.cpp)
target_link_libraries(laserUtilsServer ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
add_dependencies(laserUtilsServer ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(laser_utils_viz ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
add_dependencies(laser_utils_viz ${PROJECT_NAME}_generate_messages_cpp)
