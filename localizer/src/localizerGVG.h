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
#include "localizer/GVGmap.h"
#include "localizer/LoadLocalizerMap.h"
#include "localizer/InitLoadMapTransform.h"

#include <Eigen/Dense>
using namespace Eigen;
using namespace tf;
using namespace std;

class localizerGVG {

 public:
  localizerGVG(std::string& robot_name);
  void handleOdom(const nav_msgs::Odometry::ConstPtr& msg);
  bool processStart(localizer::StartFilter::Request  &req,
			       localizer::StartFilter::Response &res);
  bool processMeetpoint(localizer::UpdateFilter::Request  &req,
			localizer::UpdateFilter::Response &res);
  bool processMaxUnc(localizer::MaxUncertainty::Request  &req,
		     localizer::MaxUncertainty::Response &res);
  bool processMinUnc(localizer::MinUncertainty::Request  &req,
		     localizer::MinUncertainty::Response &res);
  bool loadLocalizerMap(localizer::LoadLocalizerMap::Request &req,
         localizer::LoadLocalizerMap::Response &res);
  bool initLoadMapTransform(localizer::InitLoadMapTransform::Request &req, 
         localizer::InitLoadMapTransform::Response &res);

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
  ros::ServiceServer  loadMapService;
  ros::ServiceServer  initLoadMapTransService;

  double _Wvv;
  double _Wvw;
  double _Www;

  bool filterOn;
  bool initOdom;
  bool mapLoadLocalization;
  double oldX;
  double oldY;
  double oldYaw;
  ros::Time oldStamp;
  localizer::GVGmap theMap;
  tf::Transformer transformer_;

  vector<int> idList;
  // List of nodes we added before getting a fix on robot pose
  vector<int> preFixIdList;

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

  double bearing_angle;

  // EKF variables
  int      nL; // Number of Landmarks
  Matrix2d Qr; // Odometry noise covariance
  Matrix3d R;  // The sensing error for the meetpoints
  Matrix2d J;  // Basic matrix for computations [0 -1; 1 0]

  Matrix3d CF; // Cumulative propagation F function

  
  VectorXd X; // The State Vector
  MatrixXd P; // The Covariance Matrix

  VectorXd X_init;
  MatrixXd P_init;
};

#endif
