#include <ros/ros.h>
#include "LaserUtils.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  std::string robot_name = "indoor";
  ros::init(argc, argv, robot_name);
  LaserUtils lu(robot_name);
  ros::spin();
  return 0;
}
