#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include "gvg_planner/SelectBearing.h"

class Planner {

 public:
  Planner();
  bool SelectBearingGVG(gvg_planner::SelectBearing::Request& req, gvg_planner::SelectBearing::Response& res);


 private:

  int state;
  std::vector<int> vertex_list;
  int target;
  double absoluteToRelativeAngle(double robot_angle, double absolute_angle);

  ros::NodeHandle nh;
  ros::ServiceServer select_bearing_srv;
  ros::ServiceClient retrieve_bearings_cln;
  ros::ServiceClient retrieve_path_cln;
  ros::ServiceClient min_uncertainty_cln;
  ros::ServiceClient max_uncertainty_cln;
  ros::ServiceClient check_relocalize_cln;
};

#endif
