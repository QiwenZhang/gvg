#ifndef ROBOTSIM_H
#define ROBOTSIM_H

#include <ros/ros.h>
#include <robot_node/RelRotate.h>
#include <robot_node/Brake.h>
#include <robot_node/RelTranslate.h>
#include <robot_node/Follow_Wall.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include "PIDController.h"

class robotSim {

 public:
  
  robotSim(std::string& name, int id); 
  ~robotSim(); 

  /* Return the id of the robot, in case there are many */
  int getID() { return id; }
  
  /* Accepts dy, dtheta as errors and drives the robot to a direction
     that will minimize those errors */
  void follow_wall(const robot_node::Follow_Wall::ConstPtr& msg);
  
  /* Reads up the odometry of the robot from Stage  */
  void handleOdometry(const nav_msgs::Odometry::ConstPtr& msg);
  
  /* Motion command: tVel, the translational velocity is in m/sec 
     and aVel, the angular velocity is in deg/sec */ 
  void move(double tVel, double aVel);
  
  /* Move forward by dist meters */
  bool relTranslate(robot_node::RelTranslate::Request& req, 
		    robot_node::RelTranslate::Response& res);

  /* Rotate by dtheta in degrees in place */
  bool relRotate(robot_node::RelRotate::Request& req, 
		 robot_node::RelRotate::Response& res);
			
  /* Stop the robot */
  bool stop(robot_node::Brake::Request& req,
	    robot_node::Brake::Response& res);
    
  double distance(geometry_msgs::Point& a, geometry_msgs::Point& b);

 private:
  int id;
  std::string name;
  nav_msgs::Odometry odom;
  PIDController *ang_vel_control;
  PIDController *dy_control;

  ros::NodeHandle    nh;
  ros::Publisher     movePub;
  ros::Subscriber    odomSub;
  ros::Subscriber    wfSub;
  ros::ServiceServer relRotSrv;
  ros::ServiceServer relTranSrv;
  ros::ServiceServer brakeSrv;
};

#endif	// ROBOTSIM_H
