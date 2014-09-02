#include <iostream>
#include <stdio.h>
#include <limits>
#include <cassert>
#include <cmath>
#include <algorithm>
#include "BasicGeometry.h"
#include "LaserUtils.h"

using namespace std;

LaserUtils::LaserUtils(std::string& robot_name) {
  nh.getParam("/" + robot_name + "/laser_utils/laser_utils_server/robot_diam", ROBOT_DIAM);
  nh.getParam("/" + robot_name + "/laser_utils/laser_utils_server/laser_max_range", LASER_MAX_RANGE);
  laserSub   = nh.subscribe("/indoor/base_scan", 1, &LaserUtils::handleLaserScan, this);
  allObsPub  = nh.advertise<laser_node::Obstacles>("/indoor/laser_utils/closest_obstacles", 1);
}

double LaserUtils::getRange(const sensor_msgs::LaserScan::ConstPtr& msg, int i) { 
  float r = msg->ranges.at(i);
  assert(i >= 0 && i <= countScans - 1);
  if (r > LASER_MAX_RANGE) return msg->range_max;
  return r;
}

double LaserUtils::getBearing(const sensor_msgs::LaserScan::ConstPtr& msg, int i) { 
  double theta = i*msg->angle_increment + msg->angle_min;
  assert(msg->angle_max > msg->angle_min);
  assert(msg->angle_max > M_PI/2.0 && msg->angle_min < -M_PI/2.0);
  assert(i >= 0 && i <= countScans - 1);
  assert(theta >= msg->angle_min && theta <= msg->angle_max);
  return theta;
}

geometry_msgs::Point32 LaserUtils::toPoint32(const sensor_msgs::LaserScan::ConstPtr& msg, int i) {
  geometry_msgs::Point32 p;
  double r = getRange(msg, i);
  double theta = getBearing(msg, i);
  p.x = r * cos(theta);
  p.y = r * sin(theta);
  return p;
}

void LaserUtils::populate_window_around_object(int index, 
					       std::vector<geometry_msgs::Point32>& window_before, 
					       std::vector<geometry_msgs::Point32>& window_after, 
					       int size, double& min_distance, 
					       std::vector<int>& except, 
					       const sensor_msgs::LaserScan::ConstPtr& msg,
					       bool full_size_window) {

  int size_before = (int) except.size();
  min_distance = getRange(msg, index);
  assert(std::abs(-0.22) == 0.22);
  assert(std::min(-0.22, -0.23) == -0.23);
  assert(std::max(-0.22, -0.23) == -0.22);
  
  for (int j = index + 1; j < std::min(index + size, countScans); j++) {
    if (getRange(msg, j) < msg->range_max &&
	std::abs(getRange(msg, j) - getRange(msg, index)) <= MIN_RANGE_JUMP && 
	find(except.begin(), except.end(), j) == except.end()) {

      window_after.push_back(toPoint32(msg, j));
      if (getRange(msg, j) < min_distance) {
	min_distance = getRange(msg, j);
      }
	
      except.push_back(j);
    } else if (!full_size_window){
      break;
    }
  }

  except.push_back(index);

  for (int j = index - 1; j >= std::max(index - size, 0); j--) {
    if (getRange(msg, j) < msg->range_max &&
      std::abs(getRange(msg, j) - getRange(msg, index)) <= MIN_RANGE_JUMP && 
      find(except.begin(), except.end(), j) == except.end()) {

      window_before.push_back(toPoint32(msg, j));
      if (getRange(msg, j) < min_distance) {
	      min_distance = getRange(msg, j);
      }
	
      except.push_back(j);
    } else if (!full_size_window){
      break;
    }
  }
  reverse(window_before.begin(), window_before.end());
  int size_after = except.size();
  assert(size_after == (size_before + (int) window_before.size() + (int) window_after.size() + 1));
}

void LaserUtils::handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
  
  // Laser scan filtering (all scans < range_min mapped to range_max)
  sensor_msgs::LaserScan scan;
  scan.header = msg->header;
  scan.angle_min = msg->angle_min;
  scan.angle_max = msg->angle_max;
  scan.angle_increment = msg->angle_increment;
  scan.time_increment = msg->time_increment;
  scan.scan_time = msg->scan_time;
  scan.range_min = msg->range_min;
  scan.range_max = msg->range_max;
  for (int i = 0; i < (int) msg->ranges.size(); i++) {
    scan.ranges.push_back(msg->ranges.at(i));
    if (scan.ranges.at(i) < scan.range_min) {
      scan.ranges.at(i) = scan.range_max;
    }
  }
  
  countScans = std::floor((scan.angle_max - scan.angle_min)/scan.angle_increment);

  laser_node::Obstacles obs_msg;
  
  bool found_good_obstacle_set = false;
  double epsilon = 0.5;  //in meters
  double minRange = 0;

  while (minRange + epsilon < scan.range_max && !found_good_obstacle_set) {
    obs_msg.collection.clear();
    getClosestObstaclesViaAnnulus(obs_msg, epsilon, msg);
    
    if (obs_msg.collection.empty()) {
      epsilon += 0.5;
   
    } else if (obs_msg.collection.size() == 1){ 
      minRange = obs_msg.collection.at(0).min_distance;
      epsilon += 0.5;
    
    } else if (obs_msg.collection.size() == 3) {
      laser_node::Obstacles obs_msg2;
      getClosestObstaclesViaAnnulus(obs_msg2, epsilon*1.5, msg);
      if (obs_msg2.collection.size() > 3) {
        obs_msg = obs_msg2;
      }
      
      found_good_obstacle_set = true;

    } else {
      found_good_obstacle_set = true;
    }
  }
  
  std::vector<int> obstacles_to_merge;
  std::vector<int> obstacles_indices;
  // Sort all the obstacles' indices
  for (int i = 0; i < (int) obs_msg.collection.size(); i++) {
    obstacles_indices.push_back(obs_msg.collection.at(i).start_index);
    obstacles_indices.push_back(obs_msg.collection.at(i).end_index);
  }
  std::sort(obstacles_indices.begin(), obstacles_indices.end());
  // Check that all the possible bearings are indeed accessible
  for (int j = 1; j < (int) obstacles_indices.size() - 1; j += 2) {
    int current_index = obstacles_indices.at(j);
    int end_index = obstacles_indices.at(j+1);
    int next = current_index + 1;
    bool opening = false;
    while (current_index < end_index) {
      while ((getRange(msg, next) >= scan.range_max)||(getRange(msg, next) < scan.range_min)) next++;
      geometry_msgs::Point32 start = toPoint32(msg, current_index);
      geometry_msgs::Point32 end = toPoint32(msg, next);
      current_index = next;
      next = current_index + 1;
      if (norm(start, end) > ROBOT_DIAM) {
        opening = true;
        break;
      }
    }
    // No opening available, keep track of which obstacles need to be merged
    if (!opening) {
      int first, second;
      for (int k = 0; k < (int) obs_msg.collection.size(); k++) {
        if (obstacles_indices.at(j) == obs_msg.collection.at(k).end_index) first = k;
        else if (obstacles_indices.at(j+1) == obs_msg.collection.at(k).start_index) second = k;
      }
      obstacles_to_merge.push_back(first);
      obstacles_to_merge.push_back(second);
    }
  }

  if (!obstacles_to_merge.empty()) {

    std::vector<laser_node::Obstacle> collection;
    // Merge the obstacles by fixing the obstacle collection list
    for (int i = 0; i < (int) obstacles_to_merge.size(); i += 2) {
      laser_node::Obstacle first;
      if ((i > 0) && (obstacles_to_merge.at(i) == obstacles_to_merge.at(i-1))) {
        first = collection.back();
        collection.pop_back();
      }
      else first = obs_msg.collection.at(obstacles_to_merge.at(i));
      laser_node::Obstacle second = obs_msg.collection.at(obstacles_to_merge.at(i+1));
      laser_node::Obstacle result;
      result.start_index = first.start_index;
      result.end_index = second.end_index;
      if (first.min_distance < second.min_distance) {
        result.min_distance = first.min_distance;
        result.min_bearing = first.min_bearing;
        result.min_index = first.min_index;
        result.closest_point = first.closest_point;
      } else {
        result.min_distance = second.min_distance;
        result.min_bearing = second.min_bearing;
        result.min_index = second.min_index;
        result.closest_point = second.closest_point;
      }
      for (int j = 0; j < (int) first.surface.size(); j++) result.surface.push_back(first.surface.at(j));
      int index = first.end_index + 1;
      while (index < second.start_index) {
        geometry_msgs::Point32 p;
        p = toPoint32(msg, index);
        result.surface.push_back(p);
        index++;
      }
      for (int j = 0; j < (int) second.surface.size(); j++) result.surface.push_back(second.surface.at(j));
      collection.push_back(result);
    }

    // Add in all the unmodified obstacles and sort
    for (int i = 0; i < (int) obs_msg.collection.size(); i++) {
      bool unmodified = true;
      for (int j = 0; j < (int) obstacles_to_merge.size(); j++) {
        if (obstacles_to_merge.at(j) == i) {
          unmodified = false;
          break;
        }
      }
      if (unmodified) collection.push_back(obs_msg.collection.at(i));
    }

    // Fix the obstacles for sharp corners and overwrite the obstacles collection (to avoid detecting non-existent meetpoint)
    if (obs_msg.collection.size() - (obstacles_to_merge.size()/2) > 1) {
      obs_msg.collection.clear();
      obs_msg.collection.assign(collection.begin(), collection.end());
      std::sort(obs_msg.collection.begin(), obs_msg.collection.end(), bind( &laser_node::Obstacle::start_index, _1 ) < bind( &laser_node::Obstacle::start_index, _2 ));  
    }
    // Otherwise, we have a case where we have a very distant endpoint (so we want to follow GVG until we find the end of the endpoint) -- we store the merged obstacles
    else {
      obs_msg.merged_collection.clear();
      obs_msg.merged_collection.assign(collection.begin(), collection.end());
      std::sort(obs_msg.merged_collection.begin(), obs_msg.merged_collection.end(), bind( &laser_node::Obstacle::start_index, _1 ) < bind( &laser_node::Obstacle::start_index, _2 ));  
    }
  }

  if (obs_msg.collection.size() == 1) {
    obs_msg.merged_collection.assign(obs_msg.collection.begin(), obs_msg.collection.end());
  }
  
  obs_msg.countScans = countScans;
  obs_msg.seq = scan.header.seq;
  obs_msg.stamp = scan.header.stamp;
  obs_msg.angle_min = scan.angle_min;
  obs_msg.angle_max = scan.angle_max;
  obs_msg.angle_increment = scan.angle_increment;
  allObsPub.publish(obs_msg);
  ros::spinOnce();
}


/* Clusters laser scans into obstacles using an annulus around the closest obstacle. */
void LaserUtils::getClosestObstaclesViaAnnulus(laser_node::Obstacles& annulus_obstacles, double epsilon, 
					       const sensor_msgs::LaserScan::ConstPtr& msg) {
  
  double minRange = msg->range_max + 1;
  for (int i = 0; i < countScans; i++) {
    if (getRange(msg, i) < minRange) {
      minRange = getRange(msg, i);
    }
  }

  int end_index = 0;
  while (end_index < countScans) {
    
    if (end_index < countScans && getRange(msg, end_index) < minRange + epsilon) {  // new obstacle starts
     
      laser_node::Obstacle obs;
      obs.start_index  = end_index;
      obs.min_distance = msg->range_max + 1;
      obs.min_bearing  = msg->angle_min - 1;
      obs.min_index    = -1;
      obs.end_index    = obs.start_index;
          
      do {
        double bearing = getBearing(msg, end_index);
        double range   = getRange(msg, end_index);
        geometry_msgs::Point32 p = toPoint32(msg, end_index);
        obs.surface.push_back(p);
        obs.end_index = end_index;

        if (range < obs.min_distance) {
          obs.min_distance = range;
          obs.min_bearing = bearing;
          obs.min_index = end_index;
          obs.closest_point = p;
        }
              
        end_index++;
      } while (end_index < countScans && getRange(msg, end_index) < minRange + epsilon);

      assert(obs.start_index <= obs.end_index);
      
      if ((int) obs.surface.size() >= 2 && annulus_obstacles.collection.empty()) {
	      annulus_obstacles.collection.push_back(obs);

      } else if ((int) obs.surface.size() >= 2) {

        if (norm(annulus_obstacles.collection.back().surface.back(), obs.surface.front()) < ROBOT_DIAM) {
          // robot can't go through, consider them one continuous obstacle
          assert (annulus_obstacles.collection.back().end_index < obs.start_index);
          if (annulus_obstacles.collection.back().min_distance > obs.min_distance) {
            annulus_obstacles.collection.back().min_distance = obs.min_distance;
            annulus_obstacles.collection.back().min_bearing = obs.min_bearing;
            annulus_obstacles.collection.back().closest_point = obs.closest_point;
            annulus_obstacles.collection.back().min_index = obs.min_index;
          }
          
          for (int i = annulus_obstacles.collection.back().end_index + 1; i <= obs.end_index; i++) {
            annulus_obstacles.collection.back().end_index = i;
            annulus_obstacles.collection.back().surface.push_back(toPoint32(msg, i));
          }
        
        } else {
          annulus_obstacles.collection.push_back(obs);
        }
      }

    } else {
      end_index++;
    }
  }

  std::sort(annulus_obstacles.collection.begin(), annulus_obstacles.collection.end(), 
	    bind( &laser_node::Obstacle::start_index, _1 ) < bind( &laser_node::Obstacle::start_index, _2 ));
}
