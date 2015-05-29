#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include "gvg_planner/SelectBearing.h"
#include <queue>
#include <vector>
#include <Eigen/Dense>
#include "ekf_sim.h"
#include "gvg_mapper/GVGEdgeMsg.h"
#include <iostream>
#include <fstream>
#include <localizer/GVGmap.h>

using namespace Eigen;

class Relocalization_Simulation {
  
  public:
    Relocalization_Simulation(double laser_offset_x, double laser_offset_y, double Wvv, double Wvw, double Www, int nL, VectorXd state, MatrixXd cov);
    // Return the list of nodes to visit for optimal uncertainty reduction + shortest distance
    std::vector<int> find_optimal(int source, int target, double start_shortest_distance, double alpha, double &uncertainty);
    /*void propagate(geometry_msgs::PointStamped p);
    void update(int node_id, geometry_msgs::Point32 pl, geometry_msgs::Point32 measurement);
    void writeToFile();*/
    std::vector<int> backtrace(std::map<int, int> map, int source, int target);
    
  private:
    std::vector<ekf_sim> paths;
    ros::NodeHandle nh;
    ros::ServiceClient retrieve_edges_cln;
    int nL;
    VectorXd X;
    MatrixXd P;
    // The model noise covariance (linear and angular velocity) defined in localizer launch files
    double Wvv;
    double Wvw;
    double Www;
    double laser_offset_x;
    double laser_offset_y;
};

class Planner {

 public:
  Planner();
  bool SelectBearingGVG(gvg_planner::SelectBearing::Request& req, gvg_planner::SelectBearing::Response& res);
  void handleMap(const localizer::GVGmap& map);
  
 private:

  int state;
  std::vector<int> vertex_list;
  int target;
  double absoluteToRelativeAngle(double robot_angle, double absolute_angle);
  vector<int> simulateRelocalization(bool frontier_only, int node_id);

  int nL; // Number of landmarks
  bool random_reloc_target;
  VectorXd X; // The State Vector
  MatrixXd P; // The Covariance Matrix
  
  //bool repositioned;
  bool goto_min_frontier;
  bool frontier_min_uncertainty_exploration;

  ros::NodeHandle nh;
  ros::ServiceServer select_bearing_srv;
  ros::ServiceClient retrieve_bearings_cln;
  ros::ServiceClient retrieve_path_cln;
  ros::ServiceClient min_uncertainty_cln;
  ros::ServiceClient max_uncertainty_cln;
  ros::ServiceClient check_relocalize_cln;
  ros::Subscriber map_sub;
};

#endif
