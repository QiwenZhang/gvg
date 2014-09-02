#include <ros/ros.h>
#include "localizerGVG.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  std::string robot_name = "indoor";
  ros::init(argc, argv, robot_name);
  localizerGVG ekf(robot_name);
  ros::spin();
  return 0;
}
