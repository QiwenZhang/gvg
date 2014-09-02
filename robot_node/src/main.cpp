#include <ros/ros.h>
#include "robotSim.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  std::string robot_name = argv[1];
  ros::init(argc, argv, robot_name);
  robotSim robot(robot_name, 0);
  ros::spin();
  return 0;
}
