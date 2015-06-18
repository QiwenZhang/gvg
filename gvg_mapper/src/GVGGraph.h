#ifndef GVG_GRAPH_H
#define GVG_GRAPH_H

#include <ros/ros.h>
#include <string>
#include <boost/config.hpp>
#include <vector>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/PointStamped.h>
#include <laser_node/Obstacles.h>
#include "gvg_mapper/GVGNode.h"
#include "gvg_mapper/GVGEdgeMsg.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include <BasicGeometry.h>
#include "gvg_mapper/AddMeetpoint.h"
#include "gvg_mapper/AddEndpoint.h"
#include "gvg_mapper/ExtendEdge.h"
#include "gvg_mapper/RetrieveBearings.h"
#include "gvg_mapper/RetrievePath.h"
#include "gvg_mapper/CheckRelocalize.h"
#include "gvg_mapper/LoadSavedMap.h"
#include "gvg_mapper/RetrieveEdges.h"
#include "localizer/InitLoadMapTransform.h"
#include <localizer/UpdateFilter.h>
/*
 * Radius of search for an existing vertex when we are trying to perform loop closure. In m.
 */
#define SAME_VERTEX_RADIUS 0.5 


class EdgeBearing;

class UncertaintyEllipse {
 public:
  
  geometry_msgs::Point32 center;
  double halfaxis1;
  double halfaxis2;
  double angle; // Radians

  bool contains(geometry_msgs::Point32& p);

};

class GVGVertex {
 public:

  int node_id;
 
  geometry_msgs::Point32 p;

  // For meetpoints, the number of obstacles that 
  // defined it. For endpoints, 1.   
  int expected_degree;

  // Proximity to the closest obstacle
  double closest_distance; 

  double vertex_angle;

  laser_node::Obstacles surrounding_obstacles;

  // List of possible bearings (absolute angle) from vertex
  std::vector<EdgeBearing> possible_bearings;

  // True iff these two vertices are estimated to be similar 
  bool similar_vertex(GVGVertex& v);

  // edge angle diffs (Use the angle edge diff stored in bearings, relative to edge 0)
  std::vector<double> edge_angle_diffs;

};

class GVGEdge {
 public:

  int source;
  int target;

  int edge_id;
  std::vector<geometry_msgs::PointStamped> line;

  double length;

  // True iff these two edges are estimated to be similar
  bool similar_edge(GVGEdge& e);
};


typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

typedef boost::adjacency_list<boost::vecS, boost::vecS, 
                              boost::undirectedS, GVGVertex, EdgeWeightProperty> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor     Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor       Edge;

typedef boost::graph_traits<Graph>::vertex_iterator       Vertex_iter;
typedef boost::graph_traits<Graph>::edge_iterator         Edge_iter;
typedef boost::graph_traits<Graph>::adjacency_iterator    Adjacency_iter;
typedef boost::graph_traits<Graph>::out_edge_iterator     Out_edge_iter;

class EdgeBearing {
 public:
  int bearing_id;
  int edge_id;
  double edge_angle_diff;
  bool visited;

};

class GVGGraph {
 public:

  GVGGraph(void);

  /*
   * edge_angle_diffs1 should be the current vertex we are trying to match, and edge_angle_diffs2 should iterate over all the vertices in our current GVG Graph.
   * Returns bearing ID which minimizes the error that matches the two vertices. -1 if the # of bearings differ (not same vertex).
   */
  std::vector<double> edge_angle_diffs_error(std::vector<double>& edge_angle_diffs1, std::vector<double>& edge_angle_diffs2);

  /*
   * Adds a new meetpoint to the GVG graph. p is the location of the meetpoint in world  
   * coordinates, closest_distance is the distance to the (at least) three closest obstacles
   * that define the meetpoint. possible_bearings are the angles that the robot can turn by
   * to choose another GVG edge. These angles are given in laser coordinates, i.e. with respect 
   * to the x-axis, positive angles on its right, negative on its left. The angles are in degs.   
   */
  bool addMeetpoint(gvg_mapper::AddMeetpoint::Request& req, gvg_mapper::AddMeetpoint::Response& res);
  
  /*
   * Adds a new endpoint to the GVG graph. p is the location of the endpoint in world  
   * coordinates, closest_distance is the distance to the closest obstacle that defines the 
   * endpoint.    
   */
  bool addEndpoint(gvg_mapper::AddEndpoint::Request& req, gvg_mapper::AddEndpoint::Response& res);

  /*
   * The robot is currently traversing an edge, so just extend that edge in the 
   * internal representation. 
   */
  bool extendGVGEdge(gvg_mapper::ExtendEdge::Request& req, gvg_mapper::ExtendEdge::Response& res);
  
  /*
   * Service call for Planner, returns the list of possible bearings at vertex node_id.
   */
  bool retrieveBearings(gvg_mapper::RetrieveBearings::Request& req, gvg_mapper::RetrieveBearings::Response& res);

  bool retrievePath(gvg_mapper::RetrievePath::Request& req, gvg_mapper::RetrievePath::Response& res);

  bool checkRelocalize(gvg_mapper::CheckRelocalize::Request& req, gvg_mapper::CheckRelocalize::Response& res);

  bool loadSavedMap(gvg_mapper::LoadSavedMap::Request& req, gvg_mapper::LoadSavedMap::Response& res);
 
  bool retrieveEdges(gvg_mapper::RetrieveEdges::Request& req, gvg_mapper::RetrieveEdges::Response& res);

 private:
  Graph G;
  Out_edge_iter  out,    out_end;
  Adjacency_iter nhd,    nhd_end;
  int next_node_id;
  int next_edge_id;
  std::vector<int> unexploredEdgesCount;

  std_msgs::Header header;

  // Physical distance between laser range finder and robot center
  double laser_offset_x;
  double laser_offset_y;
  double relocalize_epsilon;  

  int arriving_bearing;
  bool mapLoadLocalization;

  // ROS launch params stored
  double closest_distance_threshold;
  double edge_length_threshold;
  int exploration_policy;
  int vertex_matching_policy;

  GVGEdge currentEdge;
  Vertex lastVertex;
  int lastVertexBearing;
  std::vector<GVGEdge> edges_list;
  
  Vertex nullVertex();
  Vertex addVertex(GVGVertex& v, double robot_angle, geometry_msgs::PoseWithCovariance& pose);
  std::string vertex_state(int node_id);
  std::vector<int> computeDijkstra(Vertex v, std::vector<double> &distances);
  
  // ID of the closest vertex with unexplored edges when path planning.
  int closest_unexplored_vertex;
  double last_edge_length;
  
  bool relocalize;
  double robot_angle_correction;
  std::vector<double> local_possible_bearings;
  
  // Ellipse information
  UncertaintyEllipse ellipse;
 
  /* The (at least three) objects that are going to define a meetpoint need to have
     closest point ranges that differ no more than this quantity. In m. */
  double MEETPOINT_THRESHOLD;

  /* Services provided by gvg_mapper pkg to update the graph */
  ros::NodeHandle nh;

  ros::ServiceServer add_meetpoint_srv;
  ros::ServiceServer add_endpoint_srv;
  ros::ServiceServer extend_edge_srv;
  ros::ServiceServer retrieve_bearings_srv;
  ros::ServiceServer retrieve_path_srv;
  ros::ServiceServer check_relocalize_srv;
  ros::ServiceServer load_map_srv;
  ros::ServiceServer retrieve_edges_srv;
  ros::ServiceClient process_meetpoint_cln;
  ros::ServiceClient init_load_map_transform_cln;
  ros::Publisher     gvg_node_pub;
  ros::Publisher     gvg_edge_pub;
};

double abs_angle(double theta, double phi);

double absoluteToRelativeAngle(double robot_angle, double absolute_angle);

#endif 
