#include <ros/ros.h>
#include <robot_node/RelRotate.h>
#include <robot_node/Brake.h>
#include <robot_node/RelTranslate.h>
#include <robot_node/Follow_Wall.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <limits>
#include <utility>
#include <cmath>
#include <boost/bind.hpp>
#include "robotSim.h"



robotSim::robotSim(std::string& name, int id) {
  this->id = id;
  
  double dy_kp; double dy_ki; double dy_kd; double dy_signal;
  double ang_vel_kp; double ang_vel_ki; double ang_vel_kd; double ang_vel_signal;

  nh.getParam("/" + name + "/dy_kp", dy_kp);
  nh.getParam("/" + name + "/dy_ki", dy_ki);
  nh.getParam("/" + name + "/dy_kd", dy_kd);
  nh.getParam("/" + name + "/dy_signal", dy_signal);

  nh.getParam("/" + name + "/ang_vel_kp", ang_vel_kp);
  nh.getParam("/" + name + "/ang_vel_ki", ang_vel_ki);
  nh.getParam("/" + name + "/ang_vel_kd", ang_vel_kd);
  nh.getParam("/" + name + "/ang_vel_signal", ang_vel_signal);

  
  dy_control = new PIDController(dy_signal, dy_kp, dy_kd, dy_ki);
  ang_vel_control = new PIDController(ang_vel_signal, ang_vel_kp, ang_vel_kd, ang_vel_ki);

  this->relTranSrv = nh.advertiseService("/indoor/relative_translate", &robotSim::relTranslate, this);
  this->relRotSrv = nh.advertiseService("/indoor/relative_rotate", &robotSim::relRotate, this);
  this->brakeSrv = nh.advertiseService("/indoor/brake", &robotSim::stop, this);

  this->odomSub = nh.subscribe("/indoor/gvg/odom_combined", 1, &robotSim::handleOdometry, this);
  this->wfSub   = nh.subscribe("/indoor/follow_wall", 1, &robotSim::follow_wall, this);
  this->movePub = nh.advertise<geometry_msgs::Twist>("/indoor/cmd_vel", 1);
}

robotSim::~robotSim() {
  this->relTranSrv.shutdown();
  this->relRotSrv.shutdown();
  this->brakeSrv.shutdown();
}

/* Motion command: tVel, the translational velocity is in m/sec 
   and aVel, the angular velocity is in rad/sec */ 
void robotSim::move(double tVel, double aVel) {
  geometry_msgs::Twist msg;
  msg.linear.x = tVel;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = aVel;
  movePub.publish(msg);
}

/* Accepts dy, dtheta as errors and drives the robot to a direction
   that will minimize those errors */
void robotSim::follow_wall(const robot_node::Follow_Wall::ConstPtr& msg) {
  
  static int innerLoop = 0;
  int N = 2;
  if (innerLoop % N == 0) { // execute the outer loop in the cascading control
    dy_control->update(msg->dy_in_m);
  } else {                  // execute the inner loop
    ang_vel_control->update((dy_control->signal * M_PI/180.0) -(-msg->dtheta_in_rad));
  }
  this->move(msg->lin_vel, ang_vel_control->signal);
  innerLoop = (innerLoop + 1) % N;
  
}  

/* Move forward by dx_in_m meters */
bool robotSim::relTranslate(robot_node::RelTranslate::Request& req, 
			      robot_node::RelTranslate::Response& res) {
  
  geometry_msgs::Point before = odom.pose.pose.position;
  geometry_msgs::Point after = odom.pose.pose.position;
   ros::Rate loop_rate(10); 
  while (distance(before, after) < req.dx_in_m) {
    move(req.lin_speed, 0.0);
    ros::spinOnce();
    loop_rate.sleep();
    after = odom.pose.pose.position;
  }

  res.success = true;
  return true;
}

/* TODO: bug fix, relrotate can only handle < 90 degree turns. Also ang velocity should be recalculated depending on the angle */
bool robotSim::relRotate(robot_node::RelRotate::Request& req, 
			   robot_node::RelRotate::Response& res) {

  double ang_vel = req.dtheta_rad >= 0 ? std::abs(req.ang_speed) : (-std::abs(req.ang_speed));
  ros::Duration dt(std::abs(req.dtheta_rad)/std::abs(req.ang_speed));
  ros::Time start = ros::Time::now();
  ros::Rate loop_rate(10);
  while (ros::Time::now() < start + dt) {
    move(0.0, ang_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  res.success = true;
  return true;
}

/* Stop the robot */
bool robotSim::stop(robot_node::Brake::Request& req,
		      robot_node::Brake::Response& res) {
  this->move(0, 0);
  res.success = true;
  return true;
}
  
/* Reads up the odometry of the robot from Stage  */
void robotSim::handleOdometry(const nav_msgs::Odometry::ConstPtr& msg) {
  this->odom = *msg;
}
  
double robotSim::distance(geometry_msgs::Point& a, geometry_msgs::Point& b) {
  double distSqr = (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
  return sqrt(distSqr);
}
