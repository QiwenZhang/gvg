cmake_minimum_required(VERSION 2.4.6)
project(gvg_planner)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  std_msgs  
  geometry_msgs 
  sensor_msgs 
  laser_node
  gvg_mapper
  localizer
)

add_service_files(FILES
SelectBearing.srv 
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
) 

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  ${BULLET_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

#SET ( CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -g" )

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
add_executable(gvg_planner src/main.cpp src/ekf_sim.cpp src/Planner.cpp)
#add_executable(gvg_planner src/main.cpp src/Planner.cpp)
target_link_libraries(gvg_planner ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
add_dependencies(gvg_planner ${PROJECT_NAME}_generate_messages_cpp)
