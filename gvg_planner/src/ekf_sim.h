#ifndef EKFSIM_H
#define EKFSIM_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/Point32.h>
#include "gvg_mapper/RetrievePath.h"

using namespace Eigen;
using namespace std;

class ekf_sim {
  
  public:
  ekf_sim(double Wvv, double Wvw, double Www, double a);
  ekf_sim(double Wvv, double Wvw, double Www, double a, int nL, VectorXd state, MatrixXd cov, int current_node);
  
  // EKF functions
  void copy(ekf_sim ekf);
  void propagate(geometry_msgs::PointStamped p_stamped);
  void update(int node_id, geometry_msgs::Point32 pl, geometry_msgs::Point32 measurement);
  double computeCost(int target);
  
  int last_node;
  int current_node;
  
  VectorXd X; // The State Vector
  MatrixXd P; // The Covariance Matrix
  
  double total_distance;
  double start_shortest_distance;
  double start_map_trace;
  
 private:

  ros::NodeHandle nh;
  ros::ServiceClient retrieve_path_cln;
  double oldX;
  double oldY;
  double oldYaw;
  ros::Time oldStamp;
  
  
  bool initOdom;
  
  // helper functions
  double angleDiff(double a, double b);
  double thetapp(double theta);
  double theta02p(double theta);

  void normalizeCovariance();
  Matrix2d CT(double angle);
  Matrix2d C(double angle);
  void propagateRL();

  // EKF variables
  int nL; // Number of Landmarks
  double alpha; // Balance parameter between distance vs. uncertainty for A*
  Matrix2d Qr; // Odometry noise covariance
  Matrix3d R;  // The sensing error for the meetpoints
  Matrix2d J;  // Basic matrix for computations [0 -1; 1 0]

  Matrix3d CF; // Cumulative propagation F function
  
};


#endif
