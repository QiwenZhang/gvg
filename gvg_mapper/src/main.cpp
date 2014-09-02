#include <ros/ros.h>
#include "GVGGraph.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gvg_graph");
  GVGGraph graph;
  ROS_INFO("GVG Graph ready");
  ros::spin();
  return 0;
}
