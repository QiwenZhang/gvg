cmake_minimum_required(VERSION 2.4.6)
project(gvg)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  actionlib 
  message_generation 
  actionlib_msgs
  std_msgs 
  geometry_msgs
  nav_msgs  
  sensor_msgs
  tf  
  laser_node 
  robot_node 
  gvg_planner 
  gvg_mapper 
  remapper
)

#add_message_files(FILES
#
#)

add_service_files(FILES
  Access.srv  
  SelectEdge.srv
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

add_action_files(
  FILES
    FollowEdge.action
)
#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs 
  sensor_msgs  
  geometry_msgs
  nav_msgs  
  laser_node
  gvg_mapper
)

catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  actionlib 
  std_msgs 
  tf 
  geometry_msgs 
  actionlib_msgs 
  nav_msgs 
  sensor_msgs 
  laser_node 
  robot_node 
  actionlib 
  gvg_planner 
  gvg_mapper 
  remapper 
) 
include_directories(
  ${laser_node_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

#set(ROS_COMPILE_FLAGS "-W -Wall -Wno-unused-parameter -Wextra -std=c++0x")

#common commands for building c++ executables and libraries
#rosbuild_add_gtest(test_gvg_graph src/tests/test_gvg_graph.cpp  src/GVGGraph.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(agent            src/main.cpp)
add_executable(gvg_follower     src/GVGFollowerNode.cpp src/GVGFollower.cpp)
add_executable(follow_edge_viz  src/viz.cpp)

#rosbuild_add_executable(test_meetpoint_detection  src/tests/test_detect_meetpoint.cpp src/GVGFollower.cpp)
#rosbuild_add_executable(test_scan360     src/tests/test_scan360.cpp)

target_link_libraries(agent ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

target_link_libraries(gvg_follower ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

target_link_libraries(follow_edge_viz ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

add_dependencies(agent ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(gvg_follower ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(follow_edge_viz ${PROJECT_NAME}_generate_messages_cpp)
