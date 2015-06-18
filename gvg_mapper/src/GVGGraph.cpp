#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <laser_node/Obstacles.h>
#include <gvg_mapper/GVGNode.h>
#include <gvg_mapper/GVGEdgeMsg.h>
#include "GVGGraph.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <sstream>

using namespace Eigen;
using namespace std;

double pow2(double x) { return((x)*(x)); }

/*
 * Checks if the point lies in the ellipse.
 */
bool UncertaintyEllipse::contains(geometry_msgs::Point32& p) {
  double dx = (center.x - p.x)*cos(-angle) + (center.y - p.y)*sin(-angle);
  double dy = (center.x - p.x)*-sin(-angle) + (center.y - p.y)*cos(-angle);
  double result = (dx*dx)/(halfaxis1*halfaxis1) + (dy*dy)/(halfaxis2*halfaxis2);
  return result <= 1.0;
}


/*
 * edge_angle_diffs1 should be the current vertex we are trying to match, and edge_angle_diffs2 should iterate over all the vertices in our current GVG Graph.
 * Returns bearing ID which minimizes the error that matches the two vertices. -1 if the # of bearings differ (not same vertex).
 */
vector<double> GVGGraph::edge_angle_diffs_error(std::vector<double>& edge_angle_diffs1, 
			   std::vector<double>& edge_angle_diffs2) {

  vector<double> errors;
  for (int i = 0; i < (int) edge_angle_diffs1.size(); i++) {
    double error = 0;
    for (int j = 0; j < (int) edge_angle_diffs2.size(); j++) {
      int index1 = j % ((int) edge_angle_diffs1.size());
      int index2 = (i + j) % ((int) edge_angle_diffs2.size());
      double diff = edge_angle_diffs2.at(index2) - edge_angle_diffs1.at(index1);
      error += diff*diff;
    }
    errors.push_back(error);
  }
  return errors;
}

/*
 * Returns true iff a vertex is deemed to be similar to another vertex
 * based on their local signatures.
 */
bool GVGVertex::similar_vertex(GVGVertex& v) {
  if (norm(p, v.p) > SAME_VERTEX_RADIUS) {
    return false;
  }
  return true;
}

Vertex GVGGraph::nullVertex() {
  return boost::graph_traits<Graph>::null_vertex();
}

/*
 * GVG Graph Default constructor
 */
GVGGraph::GVGGraph() {
  this->next_node_id = 0;
  this->next_edge_id = 0;
  this->closest_unexplored_vertex = -1;
  this->relocalize = false;
  this->relocalize_epsilon = 0.0;
  this->robot_angle_correction = 0.0;
  this->mapLoadLocalization = true;
  this->add_meetpoint_srv = nh.advertiseService("add_meetpoint", &GVGGraph::addMeetpoint, this);
  this->add_endpoint_srv = nh.advertiseService("add_endpoint", &GVGGraph::addEndpoint, this);
  this->extend_edge_srv = nh.advertiseService("extend_edge", &GVGGraph::extendGVGEdge, this);
  this->retrieve_bearings_srv = nh.advertiseService("retrieve_bearings", &GVGGraph::retrieveBearings, this);
  this->retrieve_path_srv = nh.advertiseService("retrieve_path", &GVGGraph::retrievePath, this);
  this->check_relocalize_srv = nh.advertiseService("check_relocalize", &GVGGraph::checkRelocalize, this);
  this->retrieve_edges_srv = nh.advertiseService("retrieve_edges", &GVGGraph::retrieveEdges, this);
  this->process_meetpoint_cln = nh.serviceClient<localizer::UpdateFilter>("/indoor/gvg/UpdateFilter");
  this->gvg_node_pub = nh.advertise<gvg_mapper::GVGNode>("node", 1);
  this->gvg_edge_pub = nh.advertise<gvg_mapper::GVGEdgeMsg>("edge", 1);
  this->lastVertex = nullVertex();
  this->load_map_srv = nh.advertiseService("load_saved_map", &GVGGraph::loadSavedMap, this);
  this->init_load_map_transform_cln = nh.serviceClient<localizer::InitLoadMapTransform>("/indoor/gvg/InitLoadMapTransform");
  nh.getParam("/indoor/gvg/agent/meetpoint_threshold", this->MEETPOINT_THRESHOLD);
  nh.getParam("/gvg_mapper/laser_distance_x", this->laser_offset_x);
  nh.getParam("/gvg_mapper/laser_distance_y", this->laser_offset_y);
  nh.getParam("/gvg_mapper/relocalize_epsilon", this->relocalize_epsilon);
  nh.getParam("/gvg_mapper/exploration_policy", this->exploration_policy);
  nh.getParam("/gvg_mapper/vertex_matching_policy", this->vertex_matching_policy);
  nh.getParam("/gvg_mapper/closest_distance_threshold", this->closest_distance_threshold);
  nh.getParam("/gvg_mapper/edge_length_threshold", this->edge_length_threshold);
}

/*
 * Adds a new meetpoint or endpoint to the GVG,
 * unless the vertex can be matched to an existing 
 * vertex in the graph. v has no node_id before its
 * signature is searched in the graph. 
 */
Vertex GVGGraph::addVertex(GVGVertex& v, double robot_angle, geometry_msgs::PoseWithCovariance& pose) {
  Vertex target = nullVertex();
  arriving_bearing = -1;

  if (vertex_matching_policy == 0 && mapLoadLocalization) {
    double closestDistance = SAME_VERTEX_RADIUS;
    Vertex_iter vertex, vertex_end;
    for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
      double dist = norm(G[*vertex].p, v.p);
      if (dist <= closestDistance && G[*vertex].similar_vertex(v)) {
        closestDistance = dist;
        target = *vertex;
      }
    }
    if (target != nullVertex()) {
      for (int i = 0; i < (int) G[target].possible_bearings.size(); i++) {
        double angle = G[target].possible_bearings.at(i).edge_angle_diff;
        if (G[target].possible_bearings.at(i).edge_angle_diff > M_PI) angle = G[target].possible_bearings.at(i).edge_angle_diff - 2*M_PI;
        double relative_angle = abs(abs_angle(G[target].vertex_angle, angle) - abs_angle(robot_angle, M_PI));
        if ((min(relative_angle, 2*M_PI-relative_angle) < M_PI/4.0)&&(num_vertices(G) > 1)) {
          arriving_bearing = G[target].possible_bearings.at(i).bearing_id;
        }
      }
    }
  }
  // Partial vertex matching with full oracle or partial oracle
  else {
    // Find the vertex with the smallest least squares error
    if (num_vertices(G) > 0) {
      // Create uncertainty ellipse centered at robot position. Any meetpoint outside of the ellipse won't be considered for the least squares error.
      ellipse.center = v.p;
      Matrix2d cov;
      cov << pose.covariance[0], pose.covariance[1], pose.covariance[6], pose.covariance[7];
      EigenSolver<MatrixXd> es(cov);
      Vector2d V1= Vector2d(std::real(es.eigenvectors().col(0)[0]),std::real(es.eigenvectors().col(0)[1]));
      Vector2d V2= Vector2d(std::real(es.eigenvectors().col(1)[0]),std::real(es.eigenvectors().col(1)[1]));
      double l1=std::real(es.eigenvalues()[0]);
      double l2=std::real(es.eigenvalues()[1]);

      if((l1 > 0.0) && (l2 > 0.0)) {
        double k = 1.0;
        ellipse.halfaxis1 = sqrt(k*(l1))*3;
        ellipse.halfaxis2 = sqrt(k*(l2))*3;
        double angle = -atan2(V1[1],V1[0]);
        if(l1 < l2) {
          double tmp = l1; l1 = l2; l2 = tmp;
          angle = -atan2(V2[1],V2[0]);
        }
        ellipse.angle = angle;
      }
      else ROS_WARN("Degenerate covariance");

      double min_error = INFINITY;
      Vertex_iter    vertex, vertex_end;
      for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
        if ((ellipse.contains(G[*vertex].p) || !mapLoadLocalization) && (abs(v.closest_distance - G[*vertex].closest_distance) <= closest_distance_threshold) && (v.expected_degree == G[*vertex].expected_degree)) {
          
          // Check if we have already traveled this edge by looking at edge signatures
          vector<int> dismissed_edges;
          vector<EdgeBearing> bearings = G[*vertex].possible_bearings;
          for (int i = 0; i < (int) bearings.size(); i++) {
            if (bearings.at(i).visited) {
              for (int j = 0; j < (int) edges_list.size(); j++) {
                if (bearings.at(i).edge_id == edges_list.at(j).edge_id) {
                  bool edge_endpoint = (lastVertex == edges_list.at(j).source && *vertex == edges_list.at(j).target) || (*vertex == edges_list.at(j).source && lastVertex == edges_list.at(j).target);
                  // make sure our last vertex is one of this edge's endpoints. also look at edge length to determine if this edge is correct
                  if (edge_endpoint && (abs(edges_list.at(j).length - currentEdge.length) < edge_length_threshold)) {
                    arriving_bearing = bearings.at(i).bearing_id;
                    target = *vertex;
                    min_error = 0;
                    break;
                  }
                  // incorrect edge
                  else dismissed_edges.push_back(bearings.at(i).bearing_id);
                }
              }
            }
            if (arriving_bearing != -1) break;
          }
                    
          if (arriving_bearing == -1) {
            // Calculate the errors for each edge angle diffs configuration
            vector<double> errors = edge_angle_diffs_error(v.edge_angle_diffs, G[*vertex].edge_angle_diffs);
            for (int i = 0; i < (int) errors.size(); i++) {
              bool dismissed = false;
              for (int j = 0; j < (int) dismissed_edges.size(); j++) {
                if (dismissed_edges.at(j) == i) {
                  dismissed = true;
                  break;
                }
              }
              // Keep the vertex with the smallest error
              if (!dismissed && errors.at(i) < min_error) {
                target = *vertex;
                arriving_bearing = i;
                min_error = errors.at(i);
              }
            }
          }
        }
      }

      if (vertex_matching_policy == 1) {
        // Confirm the correct vertex
        if (target == nullVertex() || arriving_bearing == -1) {
          ROS_WARN("No matching meetpoint found");
          target = nullVertex();
          arriving_bearing = -1;
        }
        else ROS_WARN("Vertex matching: Found vertex %d from bearing %d with least squares error %f", (int)target, arriving_bearing, min_error);
        string str = "";
        string str2 = "";

        cout << "Is this an existing vertex? (Type 'y' for yes, otherwise type anything)\n";
        getline(cin, str);
        if (str == "y") {
          cout << "Is this information correct? (Type 'n' for no, otherwise type anything)\n";
          getline(cin, str);
          // User input vertex already exists 
          if (str == "n") {
            cout << "Node ID of matching vertex?\n";
            getline(cin, str);
            stringstream stream(str);
            stream >> target;
            cout << "Bearing we arrived from?\n";
            getline(cin, str2);
            stringstream stream2(str2);
            stream2 >> arriving_bearing;
          }
        }
        else target = nullVertex();
      }
    }
  }
  
  if (num_vertices(G) > 0) {
    srand(time(NULL));
    float r = (float)rand()/(float)RAND_MAX;
    if (r <= this->relocalize_epsilon) relocalize = true;
    cout << "Current robot orientation: " << robot_angle * 180.0/M_PI << endl; 
  }

  if (target == nullVertex()) {
    v.node_id = next_node_id++;
    target = add_vertex(v, G);
    arriving_bearing = 0;
    ROS_INFO("Added new vertex %d", v.node_id);
    unexploredEdgesCount.push_back(v.expected_degree);
  } else {
    ROS_INFO("Found matching vertex %d", G[target].node_id);
    mapLoadLocalization = true;
    
    localizer::InitLoadMapTransform srv;
    srv.request.bearing_angle = abs_angle(G[target].vertex_angle, G[target].possible_bearings.at(arriving_bearing).edge_angle_diff);
    srv.request.bearing_angle = abs_angle(srv.request.bearing_angle, M_PI);
    
    if (!init_load_map_transform_cln.call(srv)) {
      ROS_WARN("Could not call localizer to pass init load map bearing information!");
    }
    
  }
  
  // TODO: Uncertain if code is useful when we have robot_pose_ekf. Fix the robot orientation relative to the arriving bearing by looking at the possible bearings angle differences
  /*if (num_vertices(G) > 0) {
    robot_angle_correction = 0;
    for (int i = 0; i < (int) v.possible_bearings.size(); i++) {
      robot_angle_correction += G[target].possible_bearings.at((i + arriving_bearing) % G[target].possible_bearings.size()).edge_angle_diff - v.possible_bearings.at(i).edge_angle_diff;
    }
    robot_angle_correction = robot_angle_correction/v.possible_bearings.size();
  }*/

  if (num_vertices(G) > 1 && lastVertex != nullVertex()) {
    G[target].possible_bearings.at(arriving_bearing).visited = true;
    // Update the vertex bearings to the newly added edge
    if (G[target].possible_bearings.at(arriving_bearing).edge_id == -1) {
      
      // Update the old vertex bearing information
      G[lastVertex].possible_bearings.at(lastVertexBearing).edge_id = currentEdge.edge_id;
      G[lastVertex].possible_bearings.at(lastVertexBearing).visited = true;
      unexploredEdgesCount.at(G[lastVertex].node_id) -= 1;
      
      G[target].possible_bearings.at(arriving_bearing).edge_id = currentEdge.edge_id;
      unexploredEdgesCount.at(G[target].node_id) -= 1;
      // Add edge to graph
      EdgeWeightProperty weight(currentEdge.length);
      add_edge(lastVertex, target, weight, G);
      add_edge(target, lastVertex, weight, G);
      currentEdge.source = G[lastVertex].node_id;
      currentEdge.target = G[target].node_id;
      edges_list.push_back(currentEdge);
      
      // Publish edge information for visualization purposes
      gvg_mapper::GVGEdgeMsg msg;
      msg.header = header;
      msg.edge_id = currentEdge.edge_id;
      msg.source = currentEdge.source;
      msg.target = currentEdge.target;
      msg.line.assign(currentEdge.line.begin(), currentEdge.line.end());
      msg.length = currentEdge.length;

      gvg_edge_pub.publish(msg);
    }
  }

  // Create new edge between this vertex and next visited vertex 
  last_edge_length = currentEdge.length;
  GVGEdge edge;
  edge.edge_id = next_edge_id++;
  edge.line.clear(); 
  currentEdge = edge;

  lastVertex = target;
  return target;
}

/*
 * Adds a new meetpoint to the GVG graph. p is the location of the meetpoint in world  
 * coordinates, closest_distance is the distance to the (at least) three closest obstacles
 * that define the meetpoint. possible_bearings are the angles that the robot can turn by
 * to choose another GVG edge. These angles are given in laser coordinates, i.e. with respect 
 * to the x-axis, positive angles on its right, negative on its left. The angles are in rads.   
 */
bool GVGGraph::addMeetpoint(gvg_mapper::AddMeetpoint::Request& req, gvg_mapper::AddMeetpoint::Response& res) {
  GVGVertex v;
  v.p.x = req.pose.pose.position.x;
  v.p.y = req.pose.pose.position.y;
  v.expected_degree = (int) req.possible_bearings.size();
  std::sort(req.possible_bearings.begin(), req.possible_bearings.end());
  assert(req.possible_bearings.at(0) <= req.possible_bearings.at(1));
  assert(req.meetpoint_obstacles.collection.size() >= 3);

  local_possible_bearings.clear();
  local_possible_bearings = req.possible_bearings;

  header = req.header;

  std::vector<double> edge_angle_diffs;

  if ( (M_PI - req.possible_bearings.back()) < M_PI - abs(req.possible_bearings.front()) ) {
    v.vertex_angle = abs_angle(req.robot_angle, req.possible_bearings.back());
    edge_angle_diffs.push_back(2*M_PI - req.possible_bearings.back() + req.possible_bearings.front());
    for (int i = 1; i < (int) req.possible_bearings.size(); i++) {
      edge_angle_diffs.push_back(std::abs(req.possible_bearings.at(i) - req.possible_bearings.at(i-1)));
    }
  }
  else {
    v.vertex_angle = abs_angle(req.robot_angle, req.possible_bearings.front());
    edge_angle_diffs.push_back(abs(req.possible_bearings.front()) - abs(req.possible_bearings.at(1)));
    for (int i = 2; i < (int) req.possible_bearings.size(); i++) {
      edge_angle_diffs.push_back(std::abs(req.possible_bearings.at(i) - req.possible_bearings.at(i-1)));
    }
    edge_angle_diffs.push_back(2*M_PI - req.possible_bearings.back() + req.possible_bearings.front());
  }

  v.edge_angle_diffs.assign(edge_angle_diffs.begin(), edge_angle_diffs.end());

  // Add the bearings to GVGVertex::possible_bearings (in relative angle to vertex_angle)
  EdgeBearing bearing;
  bearing.bearing_id = 0;
  bearing.edge_id = -1;
  bearing.edge_angle_diff = 0;
  bearing.visited = false;
  v.possible_bearings.push_back(bearing);

  for (int i = 0; i < (int) req.possible_bearings.size() - 1; i++) {
    bearing.edge_angle_diff = 0;
    bearing.bearing_id = i+1;
    bearing.edge_id = -1;
    for (int j = 0; j <= i; j++) {
      bearing.edge_angle_diff = bearing.edge_angle_diff + edge_angle_diffs.at(j);
    } 
    bearing.visited = false;
    v.possible_bearings.push_back(bearing);
  }

  v.surrounding_obstacles = req.meetpoint_obstacles;
  v.closest_distance = req.closest_distance;

  v = G[addVertex(v, req.robot_angle, req.pose)];

  // Service call to localizer for process meetpoint
  localizer::UpdateFilter srv;
  srv.request.id = v.node_id;
  srv.request.x = laser_offset_x;
  srv.request.y = laser_offset_y;
  
  double angle = v.possible_bearings.at(0).edge_angle_diff;
  if (angle > M_PI) angle -= 2*M_PI;
  double absolute = abs_angle(v.vertex_angle, angle);
  double robot_angle = v.possible_bearings.at(arriving_bearing).edge_angle_diff;
  if (robot_angle > M_PI) robot_angle -= 2*M_PI;
  double absolute_robot_angle = abs_angle(v.vertex_angle, robot_angle);
  absolute_robot_angle += M_PI;
  if (absolute_robot_angle > M_PI) absolute_robot_angle -= 2*M_PI;
  double relative_bearing = absoluteToRelativeAngle(absolute_robot_angle, absolute_robot_angle - absolute);
  relative_bearing += robot_angle_correction;
  if (relative_bearing > M_PI) relative_bearing -= 2*M_PI;
  else if (relative_bearing < -M_PI) relative_bearing += 2*M_PI;
  
  srv.request.yaw = relative_bearing;
  
  if (!process_meetpoint_cln.call(srv)) {
    ROS_WARN("Could not call process meetpoint from localizer!");
  }
  std::vector<double> X;
  X = srv.response.X;
  // Update meetpoint locations with localizer data
  for (int i = 3; i < (int) X.size(); i += 3) {
    int meetpoint_id = (i - 3)/3;
    geometry_msgs::Point32 p;
    p.x = X.at(i);
    p.y = X.at(i+1);
    G[meetpoint_id].p = p;
    G[meetpoint_id].vertex_angle = X.at(i+2);
  }

  Vertex_iter    vertex, vertex_end;
  for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    gvg_mapper::GVGNode msg;
    msg.node_id = G[*vertex].node_id;      
    msg.p       = G[*vertex].p;
    msg.degree  = G[*vertex].expected_degree;
    std::vector<gvg_mapper::EdgeBearingMsg> bearings;
    for (int i = 0; i < G[*vertex].possible_bearings.size(); i++) {
      EdgeBearing current = G[*vertex].possible_bearings.at(i);
      gvg_mapper::EdgeBearingMsg eb;
      eb.bearing_id = current.bearing_id;
      eb.edge_id = current.edge_id;
      eb.edge_angle_diff = current.edge_angle_diff;
      eb.visited = current.visited;
      bearings.push_back(eb);
    }
    msg.possible_bearings.assign(bearings.begin(), bearings.end());
    msg.edge_angle_diffs.assign(G[*vertex].edge_angle_diffs.begin(), G[*vertex].edge_angle_diffs.end());
    msg.closest_distance = G[*vertex].closest_distance;
    msg.vertex_angle = G[*vertex].vertex_angle;
    msg.surrounding_obstacles = G[*vertex].surrounding_obstacles;
    msg.header = req.header;

    gvg_node_pub.publish(msg);
    
    if (msg.node_id == v.node_id) res.node = msg;  
    
    ros::Rate r(50);
    r.sleep();
  }

  return true;
}

/*
 * Adds a new endpoint to the GVG graph. p is the location of the endpoint in world  
 * coordinates, closest_distance is the distance to the closest obstacle that defines the 
 * endpoint.    
 */
bool GVGGraph::addEndpoint(gvg_mapper::AddEndpoint::Request& req, gvg_mapper::AddEndpoint::Response& res) {
  GVGVertex v;
  v.p.x = req.pose.pose.position.x;
  v.p.y = req.pose.pose.position.y;

  v.vertex_angle = abs_angle(req.robot_angle, M_PI);
  v.expected_degree = 1;
  v.closest_distance = req.closest_distance;
  v.surrounding_obstacles = req.meetpoint_obstacles;

  EdgeBearing bearing;
  // Add the direction that the Robot came from to GVGVertex list of possible bearings
  bearing.bearing_id = 0;
  bearing.edge_id = -1;
  bearing.edge_angle_diff = 2*M_PI;
  bearing.visited = false;
  v.possible_bearings.push_back(bearing);

  v.edge_angle_diffs.push_back(2*M_PI);

  v = G[addVertex(v, req.robot_angle, req.pose)];
  
  // Service call to localizer for process meetpoint
  localizer::UpdateFilter srv;
  srv.request.id = v.node_id;
  srv.request.x = laser_offset_x;
  srv.request.y = laser_offset_y;
  
  double angle = v.possible_bearings.at(0).edge_angle_diff;
  if (angle > M_PI) angle -= 2*M_PI;
  double absolute = abs_angle(v.vertex_angle, angle);
  double robot_angle = v.possible_bearings.at(arriving_bearing).edge_angle_diff;
  if (robot_angle > M_PI) robot_angle -= 2*M_PI;
  double absolute_robot_angle = abs_angle(v.vertex_angle, robot_angle);
  absolute_robot_angle += M_PI;
  if (absolute_robot_angle > M_PI) absolute_robot_angle -= 2*M_PI;
  double relative_bearing = absoluteToRelativeAngle(absolute_robot_angle, absolute_robot_angle - absolute);
  relative_bearing += robot_angle_correction;
  if (relative_bearing > M_PI) relative_bearing -= 2*M_PI;
  else if (relative_bearing < -M_PI) relative_bearing += 2*M_PI;
  
  srv.request.yaw = relative_bearing;
  
  if (!process_meetpoint_cln.call(srv)) {
    ROS_WARN("Could not call process meetpoint from localizer!");
  }
  std::vector<double> X;
  X = srv.response.X;
  // Update meetpoint locations with localizer data
  for (int i = 3; i < (int) X.size(); i += 3) {
    int meetpoint_id = (i - 3)/3;
    geometry_msgs::Point32 p;
    p.x = X.at(i);
    p.y = X.at(i+1);
    G[meetpoint_id].p = p;
    G[meetpoint_id].vertex_angle = X.at(i+2);
  }
 
  Vertex_iter    vertex, vertex_end;
  for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    gvg_mapper::GVGNode msg;
    msg.node_id = G[*vertex].node_id;      
    msg.p       = G[*vertex].p;
    msg.degree  = G[*vertex].expected_degree;
    std::vector<gvg_mapper::EdgeBearingMsg> bearings;
    for (int i = 0; i < G[*vertex].possible_bearings.size(); i++) {
      EdgeBearing current = G[*vertex].possible_bearings.at(i);
      gvg_mapper::EdgeBearingMsg eb;
      eb.bearing_id = current.bearing_id;
      eb.edge_id = current.edge_id;
      eb.edge_angle_diff = current.edge_angle_diff;
      eb.visited = current.visited;
      bearings.push_back(eb);
    }
    msg.possible_bearings.assign(bearings.begin(), bearings.end());
    msg.edge_angle_diffs.assign(G[*vertex].edge_angle_diffs.begin(), G[*vertex].edge_angle_diffs.end());
    msg.closest_distance = G[*vertex].closest_distance;
    msg.vertex_angle = G[*vertex].vertex_angle;
    msg.surrounding_obstacles = G[*vertex].surrounding_obstacles;
    msg.header = req.header;

    gvg_node_pub.publish(msg);
    
    if (msg.node_id == v.node_id) res.node = msg;  
    
    ros::Rate r(50);
    r.sleep();
  }  
  
  return true;
}

/*
 * The robot is currently traversing an edge, so just extend that edge in the 
 * internal representation. 
 */
bool GVGGraph::extendGVGEdge(gvg_mapper::ExtendEdge::Request& req, gvg_mapper::ExtendEdge::Response& res) {
  if ((int) currentEdge.line.size() == 0) {
    currentEdge.length = 0;
  } else {
    geometry_msgs::Point32 p1;
    p1.x = req.p_stamped.point.x;
    p1.y = req.p_stamped.point.y;
    p1.z = 0.0;
    geometry_msgs::Point32 p2;
    p2.x = currentEdge.line.back().point.x;
    p2.y = currentEdge.line.back().point.y;
    p2.z = 0.0;
    currentEdge.length += norm(p1, p2);
  }

  currentEdge.line.push_back(req.p_stamped);
  
  res.success = true;
  return true;
}


/*
 * Service call for Planner, returns whether robot has random relocalize or edge length relocalize.
 */
bool GVGGraph::checkRelocalize(gvg_mapper::CheckRelocalize::Request& req, gvg_mapper::CheckRelocalize::Response& res) {
  // Check if the robot's uncertainty is too large, then relocalize.
  if (vertex_matching_policy != 0 && num_vertices(G) > 0) {
    if (relocalize) {
      relocalize = false;
      string str = "";
      cout << "relocalize (random relocalize)?\n";
      getline(cin, str);
      if (str == "y") {
        res.relocalize = 2;
        return true;
      }
    }
    if (last_edge_length >= 3.0 && last_edge_length <= max(ellipse.halfaxis1, ellipse.halfaxis2)) {
      string str = "";
      cout << "relocalize?\n";
      getline(cin, str);
      if (str == "y") {
        res.relocalize = 1;
        return true;
      }
    }
  }
  else res.relocalize = 0;
  return true;
}


/*
 * Service call for Planner, returns the list of possible bearings at vertex node_id.
 */
bool GVGGraph::retrieveBearings(gvg_mapper::RetrieveBearings::Request& req, gvg_mapper::RetrieveBearings::Response& res) {
  int node_id = req.node_id;
  // No target, so exploration mode
  if (req.target == -1) {
   
    Vertex currentVertex = nullVertex();
    Vertex_iter    vertex, vertex_end;
    for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    /* Find the vertex with node_id */
      if (G[*vertex].node_id == node_id) {
        currentVertex = *vertex;
      }
    }

    std::vector<double> bearings;
    for (int i = 0; i < (int) G[currentVertex].possible_bearings.size(); i++) {
      if (!G[currentVertex].possible_bearings.at(i).visited) {
        int index = (i - arriving_bearing);
        if (index < 0) index += local_possible_bearings.size();
        if ( (M_PI - local_possible_bearings.back()) < M_PI - abs(local_possible_bearings.front()) ) {
          index -= 1;
          if (index < 0) index += local_possible_bearings.size();
          bearings.push_back(local_possible_bearings.at(index));
        }
        else bearings.push_back(local_possible_bearings.at(index));
      }
    }

    if (bearings.size() > 0) {
      double chosen_bearing;

      /* Modify this to determine chosen bearing */
      switch (exploration_policy) {
        case 0: {
          srand (time(NULL));
          int choice = rand() % bearings.size();
          if (bearings.size() == 1) choice = 0;
          chosen_bearing = bearings[choice];
          break;
        }
        case 1: {
          chosen_bearing = bearings[bearings.size() - 1];
          break;
        }
        case 2: {
          cout << "Choose a direction for robot to explore next: ";
          string str = "";
          getline(cin, str);
          chosen_bearing = atof(str.c_str());
          break;
        }
        default: {
          chosen_bearing = bearings[0];
          break;
        }
      }
      res.chosen_bearing = chosen_bearing;
    } else res.chosen_bearing = NULL;

    /* Update the chosen bearing as visited */

    for (int i = 0; i < (int) G[currentVertex].possible_bearings.size(); i++) {
      double relative_angle;
      int index = (i - arriving_bearing);
      if (index < 0) index += local_possible_bearings.size();
      if ( (M_PI - local_possible_bearings.back()) < M_PI - abs(local_possible_bearings.front()) ) {
        index -= 1;
        if (index < 0) index += local_possible_bearings.size();
        relative_angle = local_possible_bearings.at(index);
      }
      else relative_angle = local_possible_bearings.at(index);
      if (relative_angle == res.chosen_bearing) {
        if (!G[currentVertex].possible_bearings.at(i).visited) {
          lastVertexBearing = i;
        }
        break;
      }
    }

    for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
      cout << vertex_state(G[*vertex].node_id);
    }

    return true;

  // Pathplanning mode, need to find bearing connecting node_id and target
  } else {
    int target;
    target = req.target;
    int edge_id = -1;
    double min_length = INFINITY;
    for (int i = 0; i < (int) edges_list.size(); i++) {
      bool correct_edge = false;
      if (((edges_list.at(i).source == node_id) && (edges_list.at(i).target == target)) || ((edges_list.at(i).source == target) && (edges_list.at(i).target == node_id))) correct_edge = true;
      if (correct_edge) {
        if (edges_list.at(i).length < min_length) {
          edge_id = edges_list.at(i).edge_id;
          min_length = edges_list.at(i).length;
        }
      }
    }
    // No valid edge found, meaning we have the case of endpoint as first vertex encountered.
    Vertex currentVertex = nullVertex();
    Vertex_iter vertex, vertex_end;
    for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    /* Find the vertex with node_id */
      if (G[*vertex].node_id == node_id) {
        currentVertex = *vertex;
      }
    }
    if (edge_id == -1) {
      int index = (0 - arriving_bearing);
      if (index < 0) index += local_possible_bearings.size();
      if ( (M_PI - local_possible_bearings.back()) < M_PI - abs(local_possible_bearings.front()) ) {
        index -= 1;
        if (index < 0) index += local_possible_bearings.size();
        res.chosen_bearing = local_possible_bearings.at(index);
      }
      else res.chosen_bearing = local_possible_bearings.at(index);
    }
    else {
      ROS_INFO("Edge ID between current vertex and next vertex: %d", edge_id);
      for (int i = 0; i < (int) G[currentVertex].possible_bearings.size(); i++) {
        if (G[currentVertex].possible_bearings.at(i).edge_id == edge_id) {
          double bearing;
          int index = (i - arriving_bearing);
          if (index < 0) index += local_possible_bearings.size();
          if ( (M_PI - local_possible_bearings.back()) < M_PI - abs(local_possible_bearings.front()) ) {
            index -= 1;
            if (index < 0) index += local_possible_bearings.size();
            bearing = local_possible_bearings.at(index);
          }
          else bearing = local_possible_bearings.at(index);
          res.chosen_bearing = bearing;
          break;
        }
      }
    }
    return true;
  }
}

bool GVGGraph::retrieveEdges(gvg_mapper::RetrieveEdges::Request& req, gvg_mapper::RetrieveEdges::Response& res) {
  
  for (int i = 0; i < edges_list.size(); i++) {
    GVGEdge e = edges_list.at(i);
    if (e.source == req.node_id || e.target == req.node_id) {
      gvg_mapper::GVGEdgeMsg msg;
      msg.edge_id = e.edge_id;
      msg.source = e.source;
      msg.target = e.target;
      msg.length = e.length;
      msg.line.assign(e.line.begin(), e.line.end());
      
      res.edges.push_back(msg);
    }
  }
  
  return true;
}

/*
 * Path begins from the end of the list.
 */
bool GVGGraph::retrievePath(gvg_mapper::RetrievePath::Request& req, gvg_mapper::RetrievePath::Response& res) {
  vector<int> parents;
  vector<double> distances(num_vertices(G));
  res.distance = 0.0;
  
  Vertex currentVertex = nullVertex();
  Vertex_iter vertex, vertex_end;
  for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    if (G[*vertex].node_id == req.source) {
      currentVertex = *vertex;
    }
  }
    
  parents = computeDijkstra(currentVertex, distances);
  
  
  // Choose the next target, depending on the policy.
  int target = req.target;
  if (req.target == -1) target = closest_unexplored_vertex;
  
  // Entire map has been explored.
  if (target == -1) {
    target = closest_unexplored_vertex;
    if (num_vertices(G) > 1) {
      vector<int> vertexList;
      res.vertex_list = vertexList;
    }
  }
  // Still exploring, path planning to return to target.
  else {
    res.distance = distances[target];
    if (unexploredEdgesCount.at(target) > 0) res.frontier = true;
    else res.frontier = false;
    vector<int> vertexList;
    vertexList.push_back(target);
    int current = parents[target];
    while (current != req.source) {
      vertexList.push_back(current);
      current = parents[current];
    }
    res.vertex_list = vertexList;
  }
  return true;
}

/*
 * Returns Dijkstra parents list for each vertex.
 */
vector<int> GVGGraph::computeDijkstra(Vertex v, vector<double> &distances) { 
  // Create things for Dijkstra
  vector<Vertex> parents(num_vertices(G)); // To store parents

  boost::dijkstra_shortest_paths(G, v, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
  vector<int> result;
  // Output results
  closest_unexplored_vertex = -1;
  double distance = INFINITY;
  boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
  for (boost::tie(vertexIterator, vend) = boost::vertices(G); vertexIterator != vend; ++vertexIterator) 
  {
    if ((distances[*vertexIterator] < distance)&&(unexploredEdgesCount.at(*vertexIterator) > 0)) {
      closest_unexplored_vertex = *vertexIterator;
      distance = distances[*vertexIterator];
    }
    result.push_back(parents[*vertexIterator]);
  }
  return result;
}

bool GVGGraph::loadSavedMap(gvg_mapper::LoadSavedMap::Request& req, gvg_mapper::LoadSavedMap::Response& res) {
    
  mapLoadLocalization = false;  
  int max_node_id = 0;
  for (int i = 0; i < req.nodes.size(); i++) {
    // Create vertex and copy information
    GVGVertex v;
    gvg_mapper::GVGNode node = req.nodes.at(i);
    v.node_id = node.node_id;
    if (v.node_id > max_node_id) max_node_id = v.node_id;    
    v.p = node.p;
    v.expected_degree = node.degree;
    v.closest_distance = node.closest_distance;
    int num_unexplored = 0;
    for (int j = 0; j < node.possible_bearings.size(); j++) {
      EdgeBearing eb;
      gvg_mapper::EdgeBearingMsg ebm = node.possible_bearings.at(j);
      eb.bearing_id = ebm.bearing_id;
      eb.edge_id = ebm.edge_id;
      eb.edge_angle_diff = ebm.edge_angle_diff;
      eb.visited = ebm.visited;
      // Increment num of unexplored edges
      if (!eb.visited) num_unexplored++;
      v.possible_bearings.push_back(eb);
    }
    v.edge_angle_diffs.assign(node.edge_angle_diffs.begin(), node.edge_angle_diffs.end());
    v.vertex_angle = node.vertex_angle;
    
    // Add vertex to graph
    unexploredEdgesCount.push_back(num_unexplored);
    add_vertex(v, G);
  }
  
  int max_edge_id = 0;
  for (int i = 0; i < req.edges.size(); i++) {
    GVGEdge e;
    gvg_mapper::GVGEdgeMsg em = req.edges.at(i);
    e.edge_id = em.edge_id;
    if (e.edge_id > max_edge_id) max_edge_id = e.edge_id;
    e.source = em.source;
    e.target = em.target;
    e.length = em.length;
    e.line.assign(em.line.begin(), em.line.end());
    
    // Create EWP and add edge to graph G
    EdgeWeightProperty weight(e.length);
    add_edge(e.source, e.target, weight, G);
    add_edge(e.target, e.source, weight, G);
    edges_list.push_back(e);
  }
    
  next_node_id = max_node_id + 1;
  next_edge_id = max_edge_id + 1;
  
  return true;
}

/*
 * Takes an absolute angle and a relative angle to convert to an absolute angle (within -180 to 180 degrees)
 */
double abs_angle(double theta, double phi) {
  double result;
  result = theta + phi + M_PI;
  if (result < 0) result = result + 2*M_PI;
  else if (result > 2*M_PI) result = result - 2*M_PI;
  result = result - M_PI;
  return result;
}

double absoluteToRelativeAngle(double robot_angle, double absolute_angle) {
  double relative_angle = absolute_angle;
  if (robot_angle > 0) {
    if (relative_angle < 0) relative_angle = -relative_angle;
    else {
      if (relative_angle > M_PI) relative_angle = 2*M_PI - relative_angle;
      else relative_angle = -relative_angle;
    }
  } else {
    if (relative_angle > 0) relative_angle = -relative_angle;
    else {
      if (relative_angle < -M_PI) relative_angle = -2*M_PI - relative_angle;
      else relative_angle = -relative_angle;
    }
  }
  return relative_angle;
}

/*
 * Debugging purposes. Prints out the state of a vertex in the Graph G.
 */
string GVGGraph::vertex_state(int node_id) {
  stringstream ss;
  Vertex currentVertex = nullVertex();
  Vertex_iter vertex, vertex_end;
  for (tie(vertex, vertex_end) = vertices(G); vertex != vertex_end; vertex++) {
    /* Find the vertex with node_id */
    if (G[*vertex].node_id == node_id) {
      currentVertex = *vertex;
    }
  }
  GVGVertex current = G[currentVertex];
  ss << "Vertex state for node " << node_id << " : " << "edge 0 orientation: " << current.vertex_angle*180.0/M_PI << " ";
  ss << "Unexplored Edges Count: " << unexploredEdgesCount.at(node_id) << endl;
  ss << "Angle\tBearing\tEdge\tVisited" << endl;
  for (int i = 0; i < (int) current.possible_bearings.size(); i++) {
    ss << current.possible_bearings.at(i).edge_angle_diff*180.0/M_PI << "\t" << current.possible_bearings.at(i).bearing_id << "\t" << current.possible_bearings.at(i).edge_id << "\t" << current.possible_bearings.at(i).visited << endl;
  }
  return ss.str();
}

