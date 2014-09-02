#include <ros/ros.h>
#include "GVGFollower.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gvg_follower");
  GVGFollower gvg_follower;
  ros::spin();
  return 0;
}
