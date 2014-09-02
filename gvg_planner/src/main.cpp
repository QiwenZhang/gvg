#include <ros/ros.h>
#include "Planner.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gvg_planner");
  Planner plan;
  ROS_INFO("Planner ready");
  ros::spin();
  return 0;
}
