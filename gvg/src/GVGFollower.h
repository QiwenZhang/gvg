#ifndef GVG_FOLLOWER_H
#define GVG_FOLLOWER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <boost/signals2/mutex.hpp>
#include <gvg/Access.h>
#include <gvg/FollowEdgeAction.h>
#include <gvg/SelectEdge.h>
#include <laser_node/Obstacles.h>
#include <laser_node/Obstacle.h>
#include <vector>

/*
 * Tolerance for the similarity between angles of two adjacent edges of a vertex. In rad.
 */
#define SAME_EDGE_ANGLE_DIFF_THRESHOLD 0.26179938779

/* Only one meetpoint can be detected every 10 secs */
#define MEETPOINT_PERIOD 5.0

/* The (at least three) objects that are going to define a meetpoint need to have
   closest point bearings that differ at least by this angle. In rad. */
#define MEETPOINT_BEARING_ANGLE_DIFF 0.34906585039

/* Minimum bearing difference between the two obstacles that will be used in the
   GVG midline definition, while following a GVG edge. */
#define SAME_OBJECT_MIN_BEARING 1.0471975512

/* Less than 10cm away from obstacles is considered dangerous so the robot should 
 * drastically modify its trajectory.  
 */
#define CLOSEST_ALLOWABLE_DIST 0.1  

class GVGFollower {
 public:
  GVGFollower();

  /*
   * Pairs up the closest obstacle with another obstacle, so that the robot can travel in between these 
   * two obstacles. 
   */
  void   ChooseBestMinPair(laser_node::Obstacles& obstacles, 
			   laser_node::Obstacle& left, laser_node::Obstacle& right);
  
  /*
   * Computes the errors of the robot's current pose from the GVG pose. One error is the angular
   * error which means that the robot's x-axis is not aligned with the equidistant line to the two 
   * closest points. The other error is the distance of the robot's location to the equidistant line.  
   */
  void   DivergenceFromGVG(geometry_msgs::Point32& left, geometry_msgs::Point32& right, double& dy, double& dtheta_deg);
  
  /*
   * Sends an instantaneous motion command to the robot so that it follows the
   * current GVG edge based on the given obstacle readings. 
   */
  void   InstantaneousFollowEdge(laser_node::Obstacles& obs, bool do_move, double lin_vel);


  /*
   * Navigate to the centroid of the meetpoint.
   */
  void   NavigateToMeetpoint(double lin_vel, double ang_vel);

  /*
   * Moves the robot and orients it so that it is on the GVG, based on the 
   * closest obstacle and the one that is opposite to it. 
   */
  bool   AccessGVG(gvg::Access::Request& req, gvg::Access::Response& res);
  
  /*
   * Returns yes iff the three closest obstacles (that are sufficiently far apart from each other, as  
   * specified by meetpoint_bearing_angle_diff [in rad]) have almost equal distances from the robot (up to
   * a tolerance specified by meetpoint_threshold [in m]). If this is indeed a meetpoint, possibleBearings 
   * contains a collection of possible directions [in rad] relative to its x-axis which the robot can take,
   * except from the reverse direction, i.e. 180 degrees.
   */
  bool   DetectMeetPoint(laser_node::Obstacles& obs, std::vector<double>& possibleBearings,
				double meetpoint_threshold, double meetpoint_bearing_angle_diff, 
				laser_node::Obstacles& meetpoint_obstacles);

  /*
   * Returns true iff the robot is very close to an obstacle or if there is only one obstacle.
   */
  bool   DetectEndPoint(laser_node::Obstacles& obs);
  
  /*
   * Selects one edge to follow from the given options that exist at this meetpoint.
   * Moves ahead a bit to ensure that the edge will be taken by follow edge. Returns true
   * if the motion requests to the robot were successful.
   */
  bool SelectEdge(gvg::SelectEdge::Request& req, gvg::SelectEdge::Response& res);

  /*
  void   DecideExplorationStrategy(std::vector<double>& allAdjacentEdgeBearings, 
				   std::vector<double>& edgeBearingsThatShouldBeFollowed,
				   GVGGraph& G);
  */

  

 private:

  ros::NodeHandle        nh;

  actionlib::SimpleActionServer<gvg::FollowEdgeAction>  follow_edge_srv;
  gvg::FollowEdgeFeedback                               follow_edge_fbk;
  gvg::FollowEdgeResult                                 follow_edge_res;

  boost::signals2::mutex obs_mutex; 
  laser_node::Obstacles  obstacles;
  ros::Time              timeOfLastMeetpoint;        
  nav_msgs::Odometry     odom; 
  bool                   obstacles_read;
  bool                   detect_meetpoint_bypass;

  double		 robot_angle;
  std::vector<laser_node::Obstacle> closest_obstacles;
  geometry_msgs::Point32 meetpoint;

  /* The (at least three) objects that are going to define a meetpoint need to have
   closest point ranges that differ no more than this quantity. In m. */
  double MEETPOINT_THRESHOLD;
  double LEAVE_MEETPOINT_MIN_DIST;
  double LEAVE_MEETPOINT_MAX_DIST;
  
  double laser_offset_x;
  double laser_offset_y;
 
  ros::ServiceServer     access_gvg_srv;
  ros::ServiceServer     select_edge_srv;

  ros::ServiceClient     brake_cln;
  ros::ServiceClient     rel_rotate_cln;
  ros::ServiceClient     rel_translate_cln;
  ros::Subscriber        obstacles_sub;
  ros::Subscriber        odom_sub;
  ros::Publisher         wf_pub;
  
  /* Planner service client */
  ros::ServiceClient     select_bearing_cln;

  /* Mapper service clients */
  ros::ServiceClient     add_meetpoint_cln;
  ros::ServiceClient     add_endpoint_cln;
  ros::ServiceClient     extend_edge_cln;
  
  /* Called whenever obstacles are published from the laser_utils node */
  void handle_obstacles(const laser_node::Obstacles::ConstPtr& msg);
   
  /* Called whenever odometry is published from the robot or from stage */
  void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg);

  /* Performs follow edge motions until a meetpoint or endpoint is seen.  */
  void handle_follow_edge_goal(const gvg::FollowEdgeGoalConstPtr& goal);
};
#endif
