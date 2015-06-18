#include <gvg/Access.h>
#include <gvg_planner/SelectBearing.h>
#include <robot_node/Brake.h>
#include <robot_node/RelRotate.h>
#include <robot_node/RelTranslate.h>
#include <robot_node/Follow_Wall.h>
#include <BasicGeometry.h>
#include "GVGFollower.h"
#include <gvg_mapper/AddMeetpoint.h>
#include <gvg_mapper/AddEndpoint.h>
#include <gvg_mapper/ExtendEdge.h>

#define _USE_MATH_DEFINES
using namespace std;

GVGFollower::GVGFollower():
    follow_edge_srv(nh, "follow_edge", boost::bind(&GVGFollower::handle_follow_edge_goal, this, _1), false) {
 
  access_gvg_srv    = nh.advertiseService("access", &GVGFollower::AccessGVG, this);
  select_edge_srv   = nh.advertiseService("select_edge", &GVGFollower::SelectEdge, this);

  /* Planner client */
  select_bearing_cln= nh.serviceClient<gvg_planner::SelectBearing>("/select_bearing");

  /* Mapper clients */
  add_meetpoint_cln = nh.serviceClient<gvg_mapper::AddMeetpoint>("/add_meetpoint");
  add_endpoint_cln  = nh.serviceClient<gvg_mapper::AddEndpoint>("/add_endpoint");
  extend_edge_cln   = nh.serviceClient<gvg_mapper::ExtendEdge>("/extend_edge"); 

  brake_cln         = nh.serviceClient<robot_node::Brake>("/indoor/brake");
  rel_rotate_cln    = nh.serviceClient<robot_node::RelRotate>("/indoor/relative_rotate");
  rel_translate_cln = nh.serviceClient<robot_node::RelTranslate>("/indoor/relative_translate");
  obstacles_sub     = nh.subscribe("/indoor/laser_utils/closest_obstacles", 1, &GVGFollower::handle_obstacles, this);
  odom_sub          = nh.subscribe("/indoor/gvg/odom_combined", 1, &GVGFollower::handle_odometry, this);
  wf_pub            = nh.advertise<robot_node::Follow_Wall>("/indoor/follow_wall", 1);

  this->LEAVE_MEETPOINT_MIN_DIST = 0.15;
  this->LEAVE_MEETPOINT_MAX_DIST = 0.40;
  nh.getParam("/gvg_mapper/laser_distance_x", this->laser_offset_x);
  nh.getParam("/gvg_mapper/laser_distance_y", this->laser_offset_y);
  nh.getParam("/indoor/gvg/agent/meetpoint_threshold", this->MEETPOINT_THRESHOLD);
  nh.getParam("/gvg_mapper/leave_meetpoint_min_dist", this->LEAVE_MEETPOINT_MIN_DIST);
  nh.getParam("/gvg_mapper/leave_meetpoint_max_dist", this->LEAVE_MEETPOINT_MAX_DIST);
  
  detect_meetpoint_bypass = false;
  obstacles_read = false;
  follow_edge_srv.start();
  ROS_INFO("Follow edge server is ready");
}

geometry_msgs::Point32 calculateCircumcenter(geometry_msgs::Point32& A, geometry_msgs::Point32& B, geometry_msgs::Point32& C) {
  geometry_msgs::Point32 meetpoint;
  double D = 2*(A.x*(B.y-C.y) + B.x*(C.y-A.y) + C.x*(A.y-B.y));
  meetpoint.x = ((A.x*A.x + A.y*A.y)*(B.y-C.y) + (B.x*B.x + B.y*B.y)*(C.y-A.y) + (C.x*C.x + C.y*C.y)*(A.y-B.y))/D;
  meetpoint.y = ((A.x*A.x + A.y*A.y)*(C.x-B.x) + (B.x*B.x + B.y*B.y)*(A.x-C.x) + (C.x*C.x + C.y*C.y)*(B.x-A.x))/D;
  return meetpoint;
}

void GVGFollower::handle_obstacles(const laser_node::Obstacles::ConstPtr& msg) {
  obs_mutex.lock();
  obstacles = *msg;
  obstacles_read = true;
  obs_mutex.unlock();
  
}

void GVGFollower::handle_odometry(const nav_msgs::Odometry::ConstPtr& msg) {
  odom = *msg;
}

/*
 * Moves the robot and orients it so that it is on the GVG, based on the closest obstacle 
 * and the one that is opposite to it. 
 */
bool GVGFollower::AccessGVG(gvg::Access::Request& req, gvg::Access::Response& res) {
  
  ros::Rate r(10);
  while (!obstacles_read) {
    ros::spinOnce();
    r.sleep();
  }
  
  // Step 1: find the closest obstacle in your current FOV 
  obs_mutex.lock();
  if (obstacles.collection.empty()) {
    ROS_WARN("There are no obstacles!");
    res.success = false;
    obs_mutex.unlock();
    return true;
  }
  
  int i = 0;
  while (obstacles.collection.at(i).min_bearing < 0.0) i++;
  laser_node::Obstacle forward_closest = obstacles.collection.at(i);
  unsigned int old_seq = obstacles.seq; 
  obs_mutex.unlock();

  // Step 2: Turn 180 to see if there is an obstacle that is 
  // closer to the one you found previously
  robot_node::RelRotate srv;
  srv.request.dtheta_rad = M_PI;
  srv.request.ang_speed = req.ang_vel;
  if (!rel_rotate_cln.call(srv)) {
    ROS_WARN("Could not turn the robot!");
    res.success = false;
    return true;
  }

  // wait for new obstacles
  while (obstacles.seq == old_seq) {
    ros::spinOnce();
  }
  old_seq = obstacles.seq;
  
  obs_mutex.lock();
  if (obstacles.collection.empty()) {
    ROS_WARN("There are no obstacles!");
    res.success = false;
    obs_mutex.unlock();
    return true;
  }
  
  i = 0;
  while (obstacles.collection.at(i).min_bearing < 0.0) i++;
  laser_node::Obstacle backward_closest = obstacles.collection.at(i);
  laser_node::Obstacle closest = forward_closest;
  if (forward_closest.min_distance > backward_closest.min_distance) {
    closest = backward_closest;
  }
  old_seq = obstacles.seq;
  obs_mutex.unlock();


  // Step 3: now that you found the closest obstacle around a 360 view,
  // turn to the exact opposite direction of the closest point on that obstacle.
  robot_node::RelRotate srv2;
  if (backward_closest.min_distance > forward_closest.min_distance) {
    srv2.request.dtheta_rad = forward_closest.min_bearing;
  } else {
    srv2.request.dtheta_rad = backward_closest.min_bearing <= 0 ? 
                              std::floor(M_PI + backward_closest.min_bearing) : 
                              std::floor(-M_PI + backward_closest.min_bearing);
  }
  srv2.request.ang_speed = req.ang_vel;
  if (!rel_rotate_cln.call(srv2)) {
    ROS_WARN("Could not turn the robot!");
    res.success = false;
    return true;
  }

  // wait for new obstacles
  while (obstacles.seq == old_seq) {
    ros::spinOnce();
  }
    
  obs_mutex.lock();
  if (obstacles.collection.empty()) {
    ROS_WARN("There are no obstacles!");
    res.success = false;
    obs_mutex.unlock();
    return true;
  }
  
  // find the closest obstacle opposite to the closest point you found in 2. 
  double oppositeRange = std::numeric_limits<int>::max();
  geometry_msgs::Point32 closest_opposite_point;
  for (int i = 0; i < (int) obstacles.collection.size(); i++) {
    for (int j = 0; j < (int) obstacles.collection.at(i).surface.size(); j++) {

      geometry_msgs::Point32 pij = obstacles.collection.at(i).surface.at(j);
      double bearing_from_x_axis = atan2(pij.y, pij.x);
      double distance_ij = norm(pij);

      if (std::abs(bearing_from_x_axis) <= M_PI/4.0 && distance_ij < oppositeRange) {
	      oppositeRange = distance_ij;
	      closest_opposite_point = pij;
      }
    }
  }
  obs_mutex.unlock();

  if (oppositeRange == std::numeric_limits<int>::max()) {
    ROS_WARN("There is no obstacle opposite to the closest obstacle!");
    res.success = false;
    return true;
  }
  
  // Step 4: Turn towards the middle of the closest point and the opposite obstacle point
  geometry_msgs::Point32 middle;
  middle.x = (closest_opposite_point.x - closest.min_distance)/2.0;
  middle.y = (closest_opposite_point.y + 0.0)/2.0;

  robot_node::RelRotate srv3;
  srv3.request.dtheta_rad = atan2(middle.y, middle.x);
  srv3.request.ang_speed = req.ang_vel;
  if (!rel_rotate_cln.call(srv3)) {
    ROS_WARN("Could not turn the robot!");
    res.success = false;
    return true;
  }
     
  // Step 5: move towards the middle of the closest obstacle that you found in
  // step 2 and its opposite obstacle, found in step 3. 
  
  robot_node::RelTranslate srv4;
  srv4.request.dx_in_m   = norm(middle);
  srv4.request.lin_speed = req.lin_vel;
  if (!rel_translate_cln.call(srv4)) {
    ROS_WARN("Could not translate the robot!");
    res.success = false;
    return true;
  }
 
  // Step 6: turn 90 so that you are aligned with the tangent to the closest point 
  robot_node::RelRotate srv5;
  srv5.request.dtheta_rad = -M_PI/2.0;
  srv5.request.ang_speed = req.ang_vel;
  if (!rel_rotate_cln.call(srv5)) {
    ROS_WARN("Could not turn the robot!");
    res.success = false;
    return true;
  }
  res.success = true;
  return true;
}

/*
 * Computes the errors of the robot's current pose from the GVG pose. One error is the angular
 * error which means that the robot's x-axis is not aligned with the equidistant line to the two 
 * closest points. The other error is the distance of the robot's location to the equidistant line.  
 */
void GVGFollower::DivergenceFromGVG(geometry_msgs::Point32& left, geometry_msgs::Point32& right, 
				    double& dy, double& dtheta_rad) {

  double closestObstaclesAngle = getAngleOfVector(left, right);
  dtheta_rad = M_PI/2.0 + closestObstaclesAngle;

  geometry_msgs::Point32 midp; 
  getMidpoint(left, right, midp);
  
  geometry_msgs::Point32 origin; 
  origin.x = 0;
  origin.y = 0;
  
  geometry_msgs::Point32 normal;
  normal.x = cos(closestObstaclesAngle + M_PI/2.0);
  normal.y = sin(closestObstaclesAngle + M_PI/2.0);
  
  assert(std::abs(innerProduct(left, right, origin, normal)) < 0.00001);

  if (norm(right) < norm(left)) {
    dy = distanceOfPointToLine(midp, normal, origin);
  } else {
    dy = -distanceOfPointToLine(midp, normal, origin);
  }

  follow_edge_fbk.midpoint = midp;
  follow_edge_fbk.normal   = normal;
}

/*
 * Sends an instantaneous motion command to the robot so that it follows the
 * current GVG edge based on the given obstacle readings. 
 */
void GVGFollower::InstantaneousFollowEdge(laser_node::Obstacles& msg, bool do_move, double lin_vel) {
  assert(msg.collection.size() >= 2);
  
  //Slow down
  std::vector<double>   possibleBearings;
  laser_node::Obstacles  meetpoint_obstacles;
  bool closeToMeetpoint = DetectMeetPoint(msg, possibleBearings, 3*MEETPOINT_THRESHOLD, 
					  MEETPOINT_BEARING_ANGLE_DIFF, meetpoint_obstacles);
  
  bool lastMeetpointWasALongTimeAgo = (ros::Time::now() - timeOfLastMeetpoint).toSec() > MEETPOINT_PERIOD;
 
  laser_node::Obstacle left, right;
  ChooseBestMinPair(msg, left, right);
  
  double dy, dtheta_rad;
  DivergenceFromGVG(left.closest_point, right.closest_point, dy, dtheta_rad);
  
  if (do_move) {
    robot_node::Follow_Wall fw;
    fw.dy_in_m = dy;
    fw.dtheta_in_rad = dtheta_rad;
    fw.lin_vel = (closeToMeetpoint && lastMeetpointWasALongTimeAgo) ? lin_vel/3.0 : lin_vel;
    fw.theta_of_closest_point = msg.collection.at(0).min_bearing;
    fw.range_of_closest_point = msg.collection.at(0).min_distance;
    wf_pub.publish(fw);
  }

  follow_edge_fbk.left = left;
  follow_edge_fbk.right = right;
  follow_edge_fbk.dy = dy;
  follow_edge_fbk.dtheta_in_rad = dtheta_rad;
  follow_edge_srv.publishFeedback(follow_edge_fbk);
  
  geometry_msgs::Point p; 
  p.x = odom.pose.pose.position.x; 
  p.y = odom.pose.pose.position.y;
  // We are using Z coordinate to pass the orientation of robot! Not the height of robot
  p.z = robot_angle;
  gvg_mapper::ExtendEdge extend_edge_srv;
  extend_edge_srv.request.p_stamped.header = odom.header;
  extend_edge_srv.request.p_stamped.point = p;
  
  if(!extend_edge_cln.call(extend_edge_srv)) {
    ROS_WARN("Could not call extend edge to Mapper!");
    return;
  }
}

/*
 * First we stop the robot and take a scan to compute the location of the current meetpoint.
 * We navigate to the location of the meetpoint and stop again.
 * We take another scan to confirm the location of the new meetpoint. If it is the same as the old meetpoint and we are within the threshold, we are on the meetpoint.
 */
void GVGFollower::NavigateToMeetpoint(double lin_vel, double ang_vel) {
  
  robot_node::Brake srv;
  srv.request.brake = true;
  if (!brake_cln.call(srv)) {
    ROS_WARN("Could not brake the robot!");
  }

  ros::Rate r(10);
  while (!obstacles_read) {
    ros::spinOnce();
    r.sleep();
  }

  obs_mutex.lock();

  // clear global variable
  meetpoint.x = 0.0;
  meetpoint.y = 0.0;
  
  // Retrieve odometry position
  geometry_msgs::Point32 p; 
  p.x = odom.pose.pose.position.x; 
  p.y = odom.pose.pose.position.y;

  tf::Pose pose;
  tf::poseMsgToTF(odom.pose.pose, pose);
  robot_angle = tf::getYaw(pose.getRotation());

  // Calculate the circumcenter of the triangle (see circumscribed circle on wikipedia)
  if (closest_obstacles.size() == 3) {
    meetpoint = calculateCircumcenter(closest_obstacles.at(0).closest_point, closest_obstacles.at(1).closest_point, closest_obstacles.at(2).closest_point);
  }
  // No guaranteed circumcenter, so we divide the polygon in triangles and calculate their circumcenter. Then we take centroid of circumcenter (not guaranteed to be equidistant point)
  else if (closest_obstacles.size() > 3) {
    for (int i = 0; i < (int) closest_obstacles.size(); i++) {
      geometry_msgs::Point32 current;
      current = calculateCircumcenter(closest_obstacles.at(i).closest_point, closest_obstacles.at((i+1) % closest_obstacles.size()).closest_point, closest_obstacles.at((i+2) % closest_obstacles.size()).closest_point);
      meetpoint.x += current.x;
      meetpoint.y += current.y;
    }
    meetpoint.x = meetpoint.x/closest_obstacles.size();
    meetpoint.y = meetpoint.y/closest_obstacles.size();
  }
  else {
    for (int i = 0; i < (int) closest_obstacles.size(); i++) {
      meetpoint.x += closest_obstacles.at(i).closest_point.x;
      meetpoint.y += closest_obstacles.at(i).closest_point.y;
    }
    meetpoint.x = meetpoint.x/closest_obstacles.size();
    meetpoint.y = meetpoint.x/closest_obstacles.size();
  }

  double distance = 10000;
  bool aborted = false;

  do {
    double dx = meetpoint.x - p.x;
    double dy = meetpoint.y - p.y;

    double abs_angle = atan2(dy, dx);
    double rel_angle = abs_angle - robot_angle;
    while (rel_angle > M_PI) rel_angle -= 2*M_PI;
    while (rel_angle < -M_PI) rel_angle += 2*M_PI;
    double dtheta_rad = rel_angle;

    while (abs(dtheta_rad) > M_PI/2.0) {
      robot_node::RelRotate rotate_srv;
      rotate_srv.request.dtheta_rad = dtheta_rad;
      rotate_srv.request.ang_speed = dtheta_rad >= ang_vel? ang_vel: dtheta_rad;
      if (!rel_rotate_cln.call(rotate_srv)) {
        ROS_WARN("Could not turn the robot!");
      }
      if (dtheta_rad < 0) dtheta_rad += M_PI/2.0;
      else dtheta_rad -= M_PI/2.0;
    }
    if (abs(dtheta_rad) > 0.5/180.0 * M_PI) {
      robot_node::RelRotate rotate_srv;
      rotate_srv.request.dtheta_rad = dtheta_rad;
      rotate_srv.request.ang_speed = dtheta_rad >= ang_vel? ang_vel: dtheta_rad;
      if (!rel_rotate_cln.call(rotate_srv)) {
        ROS_WARN("Could not turn the robot!");
      }
    }

    robot_angle += rel_angle;
    while (robot_angle > M_PI) robot_angle -= 2*M_PI;
    while (robot_angle < -M_PI) robot_angle += 2*M_PI;

    obs_mutex.unlock(); 
    ros::Rate rate(15);
    rate.sleep();

    while (!obstacles_read) {
      ros::spinOnce();
      r.sleep();
    }

    obs_mutex.lock();

    p.x = odom.pose.pose.position.x; 
    p.y = odom.pose.pose.position.y;
    double laser_distance = sqrt(laser_offset_x*laser_offset_x + laser_offset_y*laser_offset_y);
    p.x += cos(robot_angle)*laser_distance;
    p.y += sin(robot_angle)*laser_distance;

    dx = meetpoint.x - p.x;
    dy = meetpoint.y - p.y;

    double meetpoint_distance = sqrt(dx*dx + dy*dy);

    // Safety check for obstacle clearance in front of robot by meetpoint distance.
    bool obstacle_clearance = true;
    for (int i = 0; i < (int) obstacles.collection.size(); i++) {
      laser_node::Obstacle obs = obstacles.collection.at(i);
      if ((obs.start_index < floor(obstacles.countScans/2) - 4) && (obs.end_index > floor(obstacles.countScans/2) + 4)) {
        for (int index = floor(obstacles.countScans/2) - 4 - obs.start_index; index <= floor(obstacles.countScans/2) + 4 - obs.start_index; index++) {
          geometry_msgs::Point32 p = obs.surface.at(index);
          double dist = sqrt(p.x*p.x + p.y*p.y);
          if (dist < meetpoint_distance + CLOSEST_ALLOWABLE_DIST) obstacle_clearance = false;
        }
      }
    }

    // Abort navigation
    if (!obstacle_clearance) {
      ROS_WARN("Obstacle collision: aborted navigation.");
      closest_obstacles.clear();
      aborted = true;
      break;
    }

    robot_node::RelTranslate trans_srv;
    trans_srv.request.dx_in_m   = meetpoint_distance;
    trans_srv.request.lin_speed = 0.5*meetpoint_distance >= lin_vel ? lin_vel: 0.5*meetpoint_distance;
    if (!rel_translate_cln.call(trans_srv)) {
      ROS_WARN("Could not move the robot forward!");
    }

    robot_node::Brake brake_srv;
    brake_srv.request.brake = true;
    if (!brake_cln.call(brake_srv)) {
      ROS_WARN("Could not brake the robot!");
    }

    obs_mutex.unlock(); 
    rate.sleep();

    // Read in new laser scan
    while (!obstacles_read) {
      ros::spinOnce();
      r.sleep();
    }

    obs_mutex.lock();

    // Retrieve odometry position
    p.x = odom.pose.pose.position.x; 
    p.y = odom.pose.pose.position.y;
    p.x += cos(robot_angle)*laser_distance;
    p.y += sin(robot_angle)*laser_distance;
    dx = meetpoint.x - p.x;
    dy = meetpoint.y - p.y;
    ROS_INFO("Meetpoint: [%f %f], pose [%f %f]", meetpoint.x,meetpoint.y, p.x,p.y);
    distance = sqrt(dx*dx + dy*dy);
    if (distance > 10000) {
      ROS_WARN("Invalid distance: aborted navigation.");
      closest_obstacles.clear();
      aborted = true;
      break;
    }
    ROS_INFO("distance: %f | meetpoint: (%f,%f)", distance, meetpoint.x, meetpoint.y);
    r.sleep();
  } while (distance > MEETPOINT_THRESHOLD);

  for (int i = 0; i < (int) closest_obstacles.size(); i++) {
    double angle = closest_obstacles.at(i).min_bearing + robot_angle;
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
  }

  if (!aborted) detect_meetpoint_bypass = true;
  obs_mutex.unlock();
  
  ros::Rate rate(15);
  rate.sleep();
}

/*
 * Returns yes iff the three closest obstacles (that are sufficiently far apart from each other, as  
 * specified by meetpoint_bearing_angle_diff [in rad]) have almost equal distances from the robot (up to
 * a tolerance specified by meetpoint_threshold [in m]). If this is indeed a meetpoint, possibleBearings 
 * contains a collection of possible directions [in rad] relative to its x-axis which the robot can take,
 * except from the reverse direction, i.e. 180 degrees.
 */
bool GVGFollower::DetectMeetPoint(laser_node::Obstacles& obs, std::vector<double>& possibleBearings,
				  double meetpoint_threshold, double meetpoint_bearing_angle_diff, 
				  laser_node::Obstacles& meetpoint_obstacles) {

  std::vector<double> obstacleMinBearings;

  if (obs.collection.size() >= 3) {

    // We partition the scan range into slots of size MEETPOINT_BEARING_ANGLE_DIFF
    // For each slot we'll only accept one possible bearing, in order to avoid
    // having meetpoints that are detected by obstacles whose min bearings are very close.

    std::vector<int> occupied_slots;            
    double lp0 = obs.collection.at(0).min_distance;
    obstacleMinBearings.push_back(obs.collection.at(0).min_bearing);
    int slot = (int) std::floor(obs.collection.at(0).min_bearing / meetpoint_bearing_angle_diff); 
    occupied_slots.push_back(slot);
    occupied_slots.push_back(slot + 1);  // to ensure distance between possible bearings
    occupied_slots.push_back(slot - 1);
    meetpoint_obstacles.collection.push_back(obs.collection.at(0));

    for (int i = 1; i < (int) obs.collection.size(); i++) {
      double lpi = obs.collection.at(i).min_distance;
      slot = (int) std::floor(obs.collection.at(i).min_bearing / meetpoint_bearing_angle_diff); 

      if (std::abs(lp0 - lpi) < meetpoint_threshold && 
          find(occupied_slots.begin(), occupied_slots.end(), slot) == occupied_slots.end()) {	 

        obstacleMinBearings.push_back(obs.collection.at(i).min_bearing);
        occupied_slots.push_back(slot);
        occupied_slots.push_back(slot + 1);
        occupied_slots.push_back(slot - 1);
        meetpoint_obstacles.collection.push_back(obs.collection.at(i));
      } 
    }
  }

  // Make sure we did not miss any of the obstacles
  if (obstacleMinBearings.size() != obs.collection.size()) return false;
  
  std::sort(obstacleMinBearings.begin(), obstacleMinBearings.end());
  for (int i = 1; i <= (int) obstacleMinBearings.size(); i++) {
    double bearing;
    double first, second;
    first = obstacleMinBearings[i-1];
    second = obstacleMinBearings[i % obstacleMinBearings.size()];
    if ((first < 0 && second < 0)||(first > 0 && second > 0)) {
      bearing = (first + second)/2.0;
    }
    else {
      bearing = (first + second)/2.0;
      // We are past the two obstacles forming a possible bearing, which means that we have to take the smaller angle.
      // Note: The smaller angle should only be taken whenever the two obstacles form the direction we just came from.
      if (i == (int) obstacleMinBearings.size() && ((abs(first) + abs(second)) >= M_PI)) {
        if (bearing < 0) bearing += M_PI;
        else bearing -= M_PI;
      }
    }
    possibleBearings.push_back(bearing);
  }
  if (abs(abs(possibleBearings.back()) - M_PI) > M_PI/4.0) {
    double back = possibleBearings.back();
    possibleBearings.pop_back();
    possibleBearings.push_back(back - M_PI);
  }

  std::sort(possibleBearings.begin(), possibleBearings.end());

  return obstacleMinBearings.size() >= 3;
}


/*
 * Selects one edge to follow from the given options that exist at this meetpoint.
 * Moves ahead a bit to ensure that the edge will be taken by follow edge. Returns true
 * if the motion requests to the robot were successful.
 */
bool GVGFollower::SelectEdge(gvg::SelectEdge::Request& req, gvg::SelectEdge::Response& res) {

  // Follower service calls Planner to know which bearing to head to.
  gvg_planner::SelectBearing select_bearing_srv;
  select_bearing_srv.request.node_id = req.node_id;
  select_bearing_srv.request.robot_angle = robot_angle;
  if (!select_bearing_cln.call(select_bearing_srv)) {
    ROS_WARN("Could not request selected bearing!");
    res.success = false;
    return true;
  }
  res.selected_bearing = select_bearing_srv.response.selected_bearing;

  robot_node::RelRotate srv;
  srv.request.dtheta_rad = res.selected_bearing;
  srv.request.ang_speed =  req.ang_vel;   // 40 deg/sec
  if (!rel_rotate_cln.call(srv)) {
    ROS_WARN("Could not turn the robot!");
    res.success = false;
    return true;
  }

  ros::Rate r(15);
  r.sleep();
  ros::spinOnce();

  geometry_msgs::Point32 p; 
  p.x = odom.pose.pose.position.x; 
  p.y = odom.pose.pose.position.y;

  /* Calculate the robot's absolute orientation */
  tf::Pose pose;
  tf::poseMsgToTF(odom.pose.pose, pose);
  robot_angle = tf::getYaw(pose.getRotation());

  // Adjust Node position due to robot-laser offset
  double laser_distance = sqrt(laser_offset_x*laser_offset_x + laser_offset_y*laser_offset_y);
  p.x += cos(robot_angle)*laser_distance;
  p.y += sin(robot_angle)*laser_distance;

  double distance = 0.3;
  // calculate the distance to the midpoint between the two obstacles we are travelling to.
  for (int i = 0; i < (int) closest_obstacles.size(); i++) {
    double first = closest_obstacles.at(i).min_bearing + M_PI;
    double second = closest_obstacles.at((i+1) % closest_obstacles.size()).min_bearing + M_PI;
    if ((res.selected_bearing + M_PI >= first) && (res.selected_bearing + M_PI <= second)) {
      geometry_msgs::Point32 midpoint;
      midpoint.x = (closest_obstacles.at(i).closest_point.x + closest_obstacles.at((i+1) % closest_obstacles.size()).closest_point.x)/2;
      midpoint.y = (closest_obstacles.at(i).closest_point.y + closest_obstacles.at((i+1) % closest_obstacles.size()).closest_point.y)/2;
      double dx = midpoint.x - p.x;
      double dy = midpoint.y - p.y;
      distance = sqrt(dx*dx + dy*dy);
    }
  }

  robot_node::RelTranslate tsrv;
  if (distance < LEAVE_MEETPOINT_MIN_DIST) distance = LEAVE_MEETPOINT_MIN_DIST;
  else if (distance > LEAVE_MEETPOINT_MAX_DIST) distance = LEAVE_MEETPOINT_MAX_DIST;
  tsrv.request.dx_in_m = distance;
  tsrv.request.lin_speed = 0.5*req.lin_vel;
  if (!rel_translate_cln.call(tsrv)) {
    ROS_WARN("Could not move the robot forward!");
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

/*
 * Returns true iff the robot is very close to an obstacle or if there is only one obstacle.
 */
bool GVGFollower::DetectEndPoint(laser_node::Obstacles& obs) {
  if (obs.collection.empty()) {
    return false;
  } else {
    if (detect_meetpoint_bypass) return obs.merged_collection.size() == 1;
    else return obs.collection.at(0).min_distance < CLOSEST_ALLOWABLE_DIST || obs.collection.size() == 1;  
  }
}

/*
 * Pairs up the closest obstacle with another obstacle, so that the robot can travel in between these 
 * two obstacles. 
 */
void GVGFollower::ChooseBestMinPair(laser_node::Obstacles& obs, laser_node::Obstacle& left, laser_node::Obstacle& right) {

  bool found = false;
  assert((int) obs.collection.size() >= 2);

  if (!found) {
    double bearingOfMin = obs.collection.at(0).min_bearing; 
   
    for (int i = 1; i < (int) obs.collection.size(); i++) { // Choose the pair that has a big bearing difference

      double bearingOfCandidate = obs.collection.at(i).min_bearing; 
      bool haveBigBearingDifference = std::abs(bearingOfMin - bearingOfCandidate) >= SAME_OBJECT_MIN_BEARING; 
      
      if (obs.collection.at(i).min_index < obs.collection.at(0).min_index && haveBigBearingDifference) {
	right = obs.collection.at(i);
	left  = obs.collection.at(0);
	found = true;
	break;
      } else if (haveBigBearingDifference) {
	right = obs.collection.at(0);
	left  = obs.collection.at(i);
	found = true;
	break;
      }
    }
  }
    
  if (!found && obs.collection.at(0).min_index < obs.collection.at(1).min_index) {
    right = obs.collection.at(0);
    left  = obs.collection.at(1);

  } else if (!found) {
    right = obs.collection.at(1);
    left  = obs.collection.at(0);
  }
}

/*
 * Performs follow edge motions until a meetpoint or endpoint is seen.
 */
void GVGFollower::handle_follow_edge_goal(const gvg::FollowEdgeGoalConstPtr& goal) {
  
  // Copy information so we can overwrite it with updating parameter do_move
  bool do_move = goal->do_move;
  
  ros::Rate r(10);
  while (!obstacles_read) {
    ros::spinOnce();
    r.sleep();
  }
  std::vector<double>   possible_bearings;
  laser_node::Obstacles  meetpoint_obstacles;
  ros::Rate rate(15);

  while (ros::ok() && follow_edge_srv.isActive() && !follow_edge_srv.isPreemptRequested()) {
    nh.getParam("/indoor/gvg/agent/do_move", do_move);
    ros::spinOnce();
    possible_bearings.clear();
    meetpoint_obstacles.collection.clear();
    obs_mutex.lock();
    
    if (obstacles.collection.size() == 0) {
      std::string error = "Could not find any obstacles to do follow edge";
      ROS_INFO(error.c_str());
      follow_edge_res.success = false;
      follow_edge_res.stoppedBecause = gvg::FollowEdgeResult::FOUND_NO_OBSTACLES;
      follow_edge_srv.setAborted(follow_edge_res, error);
      obs_mutex.unlock();
      return;
    }

    geometry_msgs::Point32 p; 
    p.x = odom.pose.pose.position.x; 
    p.y = odom.pose.pose.position.y;

    /* Calculate the robot's absolute orientation */
    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    robot_angle = tf::getYaw(pose.getRotation());

    // Adjust Node position due to robot-laser offset
    double distance = sqrt(laser_offset_x*laser_offset_x + laser_offset_y*laser_offset_y);
    p.x += cos(robot_angle)*distance;
    p.y += sin(robot_angle)*distance;

    bool onEndpoint = DetectEndPoint(obstacles);
    if (onEndpoint && do_move) {
      ROS_INFO("Endpoint found!");
      follow_edge_res.success = true;
      follow_edge_res.stoppedBecause = gvg::FollowEdgeResult::FOUND_ENDPOINT;

      robot_node::Brake srv;
      srv.request.brake = true;
      if (!brake_cln.call(srv)) {
	      ROS_WARN("follow edge could not stop the robot!");
      }
      
      // detected endpoint through the navigation to meetpoint, which means we need to use the merged obstacles collection set.
      if (detect_meetpoint_bypass) {
        closest_obstacles.clear();
        obstacles.collection.clear();
        obstacles.collection.assign(obstacles.merged_collection.begin(), obstacles.merged_collection.end());
        obstacles.merged_collection.clear();
        detect_meetpoint_bypass = false;
      }

      /* Send service call to Mapper to add endpoint */
      gvg_mapper::AddEndpoint add_endpoint_srv;
      add_endpoint_srv.request.pose = odom.pose;
      add_endpoint_srv.request.pose.pose.position.x = p.x;
      add_endpoint_srv.request.pose.pose.position.y = p.y;
      add_endpoint_srv.request.closest_distance = obstacles.collection.at(0).min_distance;
      add_endpoint_srv.request.meetpoint_obstacles = obstacles;
      add_endpoint_srv.request.robot_angle = robot_angle;
      add_endpoint_srv.request.header = odom.header;

      if (!add_endpoint_cln.call(add_endpoint_srv)) {
        ROS_WARN("Could not call addEndpoint on Mapper!");
        return;
      }
      
      gvg_mapper::GVGNode result;
      result.node_id = add_endpoint_srv.response.node.node_id;
      result.p = add_endpoint_srv.response.node.p;
      result.degree = add_endpoint_srv.response.node.degree;
      result.closest_distance = add_endpoint_srv.response.node.closest_distance;
      result.edge_angle_diffs = add_endpoint_srv.response.node.edge_angle_diffs;
      result.possible_bearings = add_endpoint_srv.response.node.possible_bearings;
      result.surrounding_obstacles = add_endpoint_srv.response.node.surrounding_obstacles;

      follow_edge_res.node = result;

      follow_edge_srv.setSucceeded(follow_edge_res);
      obs_mutex.unlock();
      
      return;
    }

    // Since we moved during navigation, recalculate the different variables for each obstacle forming the meetpoint
    laser_node::Obstacles local_obstacles;
    local_obstacles.countScans = obstacles.countScans;
    local_obstacles.seq = obstacles.seq;
    local_obstacles.stamp = obstacles.stamp;
    local_obstacles.angle_min = obstacles.angle_min;
    local_obstacles.angle_max = obstacles.angle_max;
    local_obstacles.angle_increment = obstacles.angle_increment;

    for (int i = 0; i < (int) closest_obstacles.size(); i++) {
      laser_node::Obstacle obs = closest_obstacles.at(i);
      double dx = obs.closest_point.x - p.x;
      double dy = obs.closest_point.y - p.y;

      obs.min_distance = sqrt(dx*dx + dy*dy);
      double abs_angle = atan2(dy, dx);
      double rel_angle = abs_angle - robot_angle;
      while (rel_angle > M_PI) rel_angle -= 2*M_PI;
      while (rel_angle < -M_PI) rel_angle += 2*M_PI;
      obs.min_bearing = rel_angle;
      obs.closest_point.x = obs.min_distance*cos(rel_angle); 
      obs.closest_point.y = obs.min_distance*sin(rel_angle); 
      local_obstacles.collection.push_back(obs);
    }

    bool onMeetpoint = false;
    if (detect_meetpoint_bypass) onMeetpoint = DetectMeetPoint(local_obstacles, possible_bearings, 100, 
				       MEETPOINT_BEARING_ANGLE_DIFF, meetpoint_obstacles);
 
    if (onMeetpoint && do_move) {
      detect_meetpoint_bypass = false;
      ROS_INFO("Meetpoint found!");
      
      robot_node::Brake srv;
      srv.request.brake = true;
      if (!brake_cln.call(srv)) {
        ROS_WARN("follow edge could not stop the robot!");
      }

      timeOfLastMeetpoint = ros::Time::now();

      /* Send service call to Mapper to add meetpoint */
      gvg_mapper::AddMeetpoint add_meetpoint_srv;
      geometry_msgs::PoseWithCovariance pose;
      pose.pose.position.x = meetpoint.x;
      pose.pose.position.y = meetpoint.y;
      pose.covariance = odom.pose.covariance;
      add_meetpoint_srv.request.pose = pose;
      add_meetpoint_srv.request.closest_distance = local_obstacles.collection.at(0).min_distance;
      add_meetpoint_srv.request.possible_bearings = possible_bearings;
      add_meetpoint_srv.request.meetpoint_obstacles = local_obstacles;
      add_meetpoint_srv.request.robot_angle = robot_angle;
      add_meetpoint_srv.request.header = odom.header;

      if (!add_meetpoint_cln.call(add_meetpoint_srv)) {
        ROS_WARN("Could not call addMeetpoint on Mapper!");
        return;
      }
      
      gvg_mapper::GVGNode result;
      result.node_id = add_meetpoint_srv.response.node.node_id;
      result.p = add_meetpoint_srv.response.node.p;
      result.degree = add_meetpoint_srv.response.node.degree;
      result.closest_distance = add_meetpoint_srv.response.node.closest_distance;
      result.edge_angle_diffs = add_meetpoint_srv.response.node.edge_angle_diffs;
      result.possible_bearings = add_meetpoint_srv.response.node.possible_bearings;
      result.surrounding_obstacles = add_meetpoint_srv.response.node.surrounding_obstacles;

      follow_edge_res.node = result;

      obs_mutex.unlock();
      follow_edge_res.success = true;
      follow_edge_res.stoppedBecause = gvg::FollowEdgeResult::FOUND_MEETPOINT;
      follow_edge_srv.setSucceeded(follow_edge_res);
      return;
    }

    std::vector<laser_node::Obstacle> temp;
    for (int i = 0; i < (int) obstacles.collection.size(); i++) {
      bool existing = false;
      for (int j = 0; j < (int) closest_obstacles.size(); j++) {
        double angle = obstacles.collection.at(i).min_bearing + robot_angle;
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        double x = p.x + obstacles.collection.at(i).min_distance*cos(angle) - closest_obstacles.at(j).closest_point.x;
        double y = p.y + obstacles.collection.at(i).min_distance*sin(angle) - closest_obstacles.at(j).closest_point.y;
        double distance = sqrt(x*x + y*y);
        if (distance < 0.6) {
          double dx = closest_obstacles.at(j).closest_point.x - p.x;
          double dy = closest_obstacles.at(j).closest_point.y - p.y;
          double distance1 = sqrt(dx*dx + dy*dy);
          dx = obstacles.collection.at(i).closest_point.x;
          dy = obstacles.collection.at(i).closest_point.y;
          double distance2 = sqrt(dx*dx + dy*dy);
          if (distance1 < distance2) {
            temp.push_back(closest_obstacles.at(j));
          }
          else {
            geometry_msgs::Point32 new_point;
            double angle = obstacles.collection.at(i).min_bearing + robot_angle;
            while (angle > M_PI) angle -= 2*M_PI;
            while (angle < -M_PI) angle += 2*M_PI;
            new_point.x = p.x + obstacles.collection.at(i).min_distance*cos(angle);
            new_point.y = p.y + obstacles.collection.at(i).min_distance*sin(angle);
            laser_node::Obstacle obs = obstacles.collection.at(i);
            obs.closest_point = new_point;
            temp.push_back(obs);
          }       
          existing = true;
          break;
        }
      }
      if (!existing) {
        geometry_msgs::Point32 new_point;
        double angle = obstacles.collection.at(i).min_bearing + robot_angle;
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        new_point.x = p.x + obstacles.collection.at(i).min_distance*cos(angle);
        new_point.y = p.y + obstacles.collection.at(i).min_distance*sin(angle);
        laser_node::Obstacle obs = obstacles.collection.at(i);
        obs.closest_point = new_point;
        temp.push_back(obs);
      }
    }
    
    closest_obstacles.clear();
    closest_obstacles.assign(temp.begin(), temp.end());
  
    

    
      // Stop following GVG and navigate locally to current closest meetpoint
      if (do_move && closest_obstacles.size() >= 3) {
        ROS_INFO("Close to Meetpoint: starting navigation to meetpoint");
        double ang_vel;
        nh.getParam("/indoor/gvg/agent/ang_vel", ang_vel);
        obs_mutex.unlock();
        NavigateToMeetpoint(goal->lin_vel, ang_vel);
      }
      else {
        // if the robot is moved manually then follow edge will not terminate 
        // on endpoints so nothing guarantees there will be more than 2 obstacles  
        if (!do_move && obstacles.collection.size() >= 2) { 
          InstantaneousFollowEdge(obstacles, do_move, goal->lin_vel);
        } else if (do_move) {
          InstantaneousFollowEdge(obstacles, do_move, goal->lin_vel);
        }
        obs_mutex.unlock();
      }

      rate.sleep();
  
  }

  if (follow_edge_srv.isPreemptRequested()) {
    std::string error = "Follow edge goal cancelled or replaced";
    ROS_INFO(error.c_str());
    follow_edge_res.success = false;
    follow_edge_res.stoppedBecause = gvg::FollowEdgeResult::PREEMPTED;
    follow_edge_srv.setPreempted(follow_edge_res, error);
  }
}
