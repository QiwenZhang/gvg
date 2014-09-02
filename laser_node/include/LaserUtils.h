#ifndef LASER_UTILS_H
#define LASER_UTILS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <laser_node/Obstacle.h>
#include <laser_node/Obstacles.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>

/* Minimum distance that is considered a scan discontinuity between laser scans. */
#define MIN_RANGE_JUMP 0.5

class LaserUtils {

 public:
  LaserUtils(std::string& robot_name);

  void   handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg); 
    
  void   getClosestObstaclesViaAnnulus(laser_node::Obstacles& annulus_obstacles, double epsilon, 
				       const sensor_msgs::LaserScan::ConstPtr& msg);
 private:
  double getRange(const sensor_msgs::LaserScan::ConstPtr& msg, int i); 
  double getBearing(const sensor_msgs::LaserScan::ConstPtr& msg, int i); 
  geometry_msgs::Point32 toPoint32(const sensor_msgs::LaserScan::ConstPtr& msg, int i);
  
  void   populate_window_around_object(int index, 
				       std::vector<geometry_msgs::Point32>& window_before, 
				       std::vector<geometry_msgs::Point32>& window_after, 
				       int size, double& min_distance, std::vector<int>& except, 
				       const sensor_msgs::LaserScan::ConstPtr& msg,
				       bool full_size_window=false);

  ros::NodeHandle     nh;
  ros::Subscriber     laserSub;
  ros::Publisher      allObsPub;
  ros::Publisher      cornersPub;
  int                 countScans;
  double              ROBOT_DIAM;
  double              LASER_MAX_RANGE;
};

#endif
