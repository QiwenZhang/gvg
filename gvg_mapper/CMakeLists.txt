cmake_minimum_required(VERSION 2.4.6)
project(gvg_mapper)

find_package(PkgConfig REQUIRED)

pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  std_msgs  
  geometry_msgs 
  sensor_msgs 
  laser_node
  localizer
)

add_message_files(FILES
  GVGEdgeMsg.msg  
  GVGNode.msg
  EdgeBearingMsg.msg
)

add_service_files(FILES
  AddEndpoint.srv  
  AddMeetpoint.srv  
  ExtendEdge.srv  
  RetrieveBearings.srv
  RetrievePath.srv
  CheckRelocalize.srv
  LoadSavedMap.srv
  RetrieveEdges.srv
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
  laser_node
)

catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs
  geometry_msgs 
  sensor_msgs 
  laser_node 
  localizer
) 

include_directories(${laser_node_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})
#SET ( CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -g" )

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#TODO need to handle basic_geometry
#add_library(basic_geometry ../laserNode/src/BasicGeometry.cpp)

add_executable(gvg_mapper src/main.cpp src/GVGGraph.cpp)
add_executable(gvg_mapper_viz src/viz.cpp)
target_link_libraries(gvg_mapper ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
add_dependencies(gvg_mapper ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gvg_mapper_viz ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
add_dependencies(gvg_mapper_viz ${PROJECT_NAME}_generate_messages_cpp)

message("CATKIN LIBRARIES ARE:" ${catkin_LIBRARIES})
