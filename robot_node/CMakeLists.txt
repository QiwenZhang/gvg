cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(robot_node)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
#include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
)

add_message_files(FILES
  Follow_Wall.msg  
  #RelGoto.msg  
  #SensorPacket.msg
)

add_service_files(FILES
#Brake.srv   Demo.srv  Leds.srv       RelTranslate.srv  Start.srv  Tank.srv
#Circle.srv  Dock.srv  RelRotate.srv  Reset.srv         Stop.srv   Turn.srv
RelTranslate.srv  RelRotate.srv
Brake.srv
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

catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs 
  tf 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
) 

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIRS})

#SET ( CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -g" )

add_executable(robotSim     src/robotSim.cpp    src/main.cpp    src/PIDController.cpp)

#add_executable(test_translation    src/tests/test_translation.cpp)
#add_executable(test_rotation       src/tests/test_rotation.cpp)
##add_executable(test_forward_turn   src/tests/test_forward_turn.cpp)
#add_executable(test_odometry   src/tests/test_odometry.cpp)
#add_executable(test_odometry_husky src/tests/test_odometry_husky.cpp)

target_link_libraries(robotSim ${catkin_LIBRARIES} ${BULLET_LIBRARIES})

#target_link_libraries(test_translation ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
#target_link_libraries(test_rotation ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
#target_link_libraries(test_forward_turn ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
#target_link_libraries(test_odometry ${catkin_LIBRARIES} ${BULLET_LIBRARIES})
#target_link_libraries(test_odometry_husky ${catkin_LIBRARIES} ${BULLET_LIBRARIES})




