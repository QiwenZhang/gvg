#ifndef LOCALIZERGVG_H
#define LOCALIZERGVG_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include "localizer/StartFilter.h"
#include "localizer/UpdateFilter.h"
#include "localizer/MaxUncertainty.h"
#include "localizer/MinUncertainty.h"
//#include "localizer/AugmentFilter.h"
#include "localizer/GVGmap.h"

#include <Eigen/Dense>
using namespace Eigen;
using namespace tf;
using namespace std;

// The definition of noise parameters 
//The linear velocity variance
#define _Wvv (0.075)
//The angular velocity variance
#define _Www (0.0075)
//The linear/angular velocity covariance
#define _Wvw (0.0)

class localizerGVG {

 public:
  localizerGVG(std::string& robot_name);
  //void handleOdom(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
  bool processStart(localizer::StartFilter::Request  &req,
			       localizer::StartFilter::Response &res);
  //  bool processUpdate(localizer::UpdateFilter::Request  &req,
  //		     localizer::processMeetpoint::Response &res);
  //  bool processAugment(localizer::AugmentFilter::Request  &req,
  //				 localizer::AugmentFilter::Response &res);
  bool processMeetpoint(localizer::UpdateFilter::Request  &req,
			localizer::UpdateFilter::Response &res);
  bool processMaxUnc(localizer::MaxUncertainty::Request  &req,
		     localizer::MaxUncertainty::Response &res);
  bool processMinUnc(localizer::MinUncertainty::Request  &req,
		     localizer::MinUncertainty::Response &res);

  //				 localizer::AugmentFilter::Response &res);


 private:

  // ROS Variables
  ros::NodeHandle     nh;
  ros::Subscriber     odomSub;
  ros::Publisher      posePub;
  ros::Publisher      mapPub;
  ros::ServiceServer  startService;
  ros::ServiceServer  updateService;
  ros::ServiceServer  augmentService;
  ros::ServiceServer  maxUncService;
  ros::ServiceServer  minUncService;

  bool filterOn;
  bool initOdom;
  double oldX;
  double oldY;
  double oldYaw;
  ros::Time oldStamp;
  localizer::GVGmap theMap;
  tf::Transformer transformer_;

  vector<int> idList;


  // EKF functions
  void propagate(double Vm, double Wm, double dt);
  void update();

  // helper functions
  double angleDiff(double a, double b);
  double thetapp(double theta);
  double theta02p(double theta);

  bool newLandmark(int id);
  void normalizeCovariance();
  Matrix2d CT(double angle);
  Matrix2d C(double angle);
  void propagateRL();

  // EKF variables
  int      nL; // Number of Landmarks
  Matrix2d Qr; // Odometry noise covariance
  Matrix3d R;  // The sensing error for the meetpoints
  Matrix2d J;  // Basic matrix for computations [0 -1; 1 0]

  Matrix3d CF; // Cumulative propagation F function

  
  VectorXd X; // The State Vector
  MatrixXd P; // The Covariance Matrix

  //
  // The following variables are obsolete written in an early attempt to 
  // process the individual submatrices independently.
  //
  //  Vector3d Xr;         // State of the robot  
  //  vector<Vector2d> Xl; // State vector of Landmarks in world coordinates
  // Covariance sub-matrices
  //  Matrix3d Pr;            // Robot covariancs P_{rr}
  //  vector<Matrix2d> Pll;   // Each Landmark P_{{l_i} {l_i}}
  //  vector<Matrix2d> Pllcv; // Cross corr. between landmarks P_{{l_i} {l_j}}
  //  vector<MatrixXd> Prl;   // Cross corr. between landmark and robot P_{rl}


};

#endif
