#include <gvg_planner/SelectBearing.h>
#include "Planner.h"
#include <gvg_mapper/RetrieveBearings.h>
#include <gvg_mapper/RetrievePath.h>
#include <gvg_mapper/CheckRelocalize.h>
#include <localizer/MinUncertainty.h>
#include <localizer/MaxUncertainty.h>
#include <gvg_mapper/GVGEdgeMsg.h>
#include <gvg_mapper/RetrieveEdges.h>
#include <sstream>

Relocalization_Simulation::Relocalization_Simulation(double laser_offset_x, double laser_offset_y, double Wvv, double Wvw, double Www, int nL, VectorXd state, MatrixXd cov) {
  
  retrieve_edges_cln = nh.serviceClient<gvg_mapper::RetrieveEdges>("/retrieve_edges");
  
  this->laser_offset_x = laser_offset_x;
  this->laser_offset_y = laser_offset_y;
  
  this->nL = nL;
  X = VectorXd(3*nL+3);
  P = MatrixXd::Zero(3*nL+3, 3*nL+3);
  
  for(int i = 0; i < state.size(); i++) {
    X(i) = state(i);
  }
  for(unsigned int i = 0; i < cov.rows(); i++) {
    for(unsigned int j = 0; j < cov.cols(); j++) {
      P(i,j) = cov(i,j);
    }
  }
  this->Wvv = Wvv;
  this->Wvw = Wvw;
  this->Www = Www;
}

// A* search guided by uncertainty and minimizing path length
std::vector<int> Relocalization_Simulation::find_optimal(int source, int target, double start_shortest_distance, double alpha, double &uncertainty) {
  
  std::vector<ekf_sim> paths;
  
  std::set<int> open_set;
  std::set<int> closed_set;
  std::map<int, int> came_from;
  
  ekf_sim init_sim(Wvv, Wvw, Www, alpha, nL, X, P, source);
  // Push the first few directions into the stack
  open_set.insert(source);
  init_sim.last_node = source;
  init_sim.current_node = source;
  init_sim.total_distance = 0.0;
  init_sim.start_shortest_distance = start_shortest_distance;
  paths.push_back(init_sim);
  /* Iterate through all different paths
   * At every step we take one path out and propagate the edge to its destination
   */
  while (!paths.empty()) {
    
    ekf_sim current = paths.at(0);
    int min_index = 0;
    for (int i = 1; i < paths.size(); i++) {
      if (paths.at(i).computeCost(target) < current.computeCost(target)) {
        current = paths.at(i);
        min_index = i;
      }
    }
    
    //ROS_INFO("Next min cost node: %d", current.current_node);
    
    paths.erase(paths.begin() + min_index);
    // Maintain open/closed set
    open_set.erase(current.current_node);
    closed_set.insert(current.current_node);
    
    /*ROS_INFO("Open set:" );
    for (set<int>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
      cout << ' ' << *it;
    }
    cout << "\n";
    
    ROS_INFO("Closed set:" );
    for (set<int>::iterator it = closed_set.begin(); it != closed_set.end(); ++it) {
      cout << ' ' << *it;
    }
    cout << "\n";*/
    
    // Need to get all edges to current node of the path
    if (current.current_node == target) {
      cout << "Total distance travelled: " << current.total_distance << " | trace uncertainty: " << current.P.block(3, 3, 3*nL, 3*nL).trace() << endl;
      /*ofstream myfile;
      string uncertainty_output = "/home/thalassa/qzhang32/Downloads/" + temp + "/uncertainty.csv";
      myfile.open(uncertainty_output.c_str(), ios::app);
      myfile << start_shortest_distance << "," << (1.0-(current.P.block(3, 3, 3*nL, 3*nL).trace()/current.start_map_trace)) << "\n";
      myfile.close();
      string distance_output = "/home/thalassa/qzhang32/Downloads/" + temp + "/distance.csv";
      myfile.open(distance_output.c_str(), ios::app);
      myfile << start_shortest_distance << "," << current.total_distance << "\n";*/
      uncertainty = current.P.block(3, 3, 3*nL, 3*nL).trace();
      return backtrace(came_from, source, target);
    }
    
    gvg_mapper::RetrieveEdges edges_srv;
    edges_srv.request.node_id = current.current_node;
    if (!retrieve_edges_cln.call(edges_srv)) {
      ROS_WARN("Could not check relocalize from Mapper!");
    }
    vector<gvg_mapper::GVGEdgeMsg> current_edges = edges_srv.response.edges;
    
    // Enqueue all the other directions we can go, if we have never visited those nodes
    for (int i = 0; i < current_edges.size(); i++) {
      //ROS_INFO("source %d target %d", current_edges.at(i).source, current_edges.at(i).target);
      // Check that its not part of closed set
      gvg_mapper::GVGEdgeMsg traversing_edge = current_edges.at(i);
      int next_node;
      bool forward;
      if (traversing_edge.target == current.current_node) {
        forward = false;
        next_node = traversing_edge.source;
        std::set<int>::iterator it;
        it = closed_set.find(next_node);
        if (it != closed_set.end()) {
          continue;
        }
      }
      else {
        forward = true;
        next_node = current_edges.at(i).target;
        std::set<int>::iterator it;
        it = closed_set.find(traversing_edge.target);
        if (it != closed_set.end()) {
          continue;
        }
      }
      //cout << "adding node " << next_node << endl;
      ekf_sim temp(Wvv, Wvw, Www, alpha);
      temp.copy(current);
      
      // Calculate how much rotation to reach the edges orientation
      double current_angle = X(2);
      double target_angle;
      if (forward) target_angle = traversing_edge.line.at(0).point.z;
      else {
        target_angle = traversing_edge.line.at(traversing_edge.line.size() - 1).point.z + M_PI;
        if (target_angle > M_PI) target_angle -= 2*M_PI;
      }
      
      double ang_vel = 0.0;
      nh.getParam("/indoor/gvg/agent/ang_vel", ang_vel);
      
      double rotation = target_angle - current_angle;
      // To rotate smallest angle, just follow the sign of rotation
      if (fabs(rotation) < M_PI) {
        double time = fabs(rotation/ang_vel);
        int steps = floor(time*10);
        for (int j = steps; j > 0; j--) {
          geometry_msgs::PointStamped ps;
          if (forward) {
            ps = traversing_edge.line.at(0);
            int timesteps = j;
            while (timesteps >= 10) {
              ps.header.stamp.sec -= 1;
              timesteps -= 10;
            }
            int nsecs = ps.header.stamp.nsec - timesteps*100000000;
            while (nsecs < 0) {
              nsecs += 1000000000;
              ps.header.stamp.sec -= 1;
            }
            ps.header.stamp.nsec = nsecs;
          }
          else { 
            ps = traversing_edge.line.at(traversing_edge.line.size() - 1);
            int timesteps = j;
            while (timesteps >= 10) {
              ps.header.stamp.sec += 1;
              timesteps -= 10;
            }
            int nsecs = ps.header.stamp.nsec + timesteps*100000000;
            while (nsecs > 1000000000) {
              nsecs -= 1000000000;
              ps.header.stamp.sec += 1;
            }
            ps.header.stamp.nsec = nsecs;
          }

          ps.point.z = current_angle + (steps-j)*ang_vel/10;
          while (ps.point.z < -M_PI) ps.point.z += 2*M_PI;
          while (ps.point.z > M_PI) ps.point.z -= 2*M_PI;
          temp.propagate(ps);
        }
      }
      // Following sign of rotation will be larger angle, so follow negative sign
      else {
        rotation = 2*M_PI - fabs(rotation);
        double time = rotation/ang_vel;
        int steps = floor(time*10);
        for (int j = steps; j > 0; j--) {
          geometry_msgs::PointStamped ps;
          if (forward) {
            ps = traversing_edge.line.at(0);
            int timesteps = j;
            while (timesteps >= 10) {
              ps.header.stamp.sec -= 1;
              timesteps -= 10;
            }
            int nsecs = ps.header.stamp.nsec - timesteps*100000000;
            while (nsecs < 0) {
              nsecs += 1000000000;
              ps.header.stamp.sec -= 1;
            }
            ps.header.stamp.nsec = nsecs;
          }
          else { 
            ps = traversing_edge.line.at(traversing_edge.line.size() - 1);
            int timesteps = j;
            while (timesteps >= 10) {
              ps.header.stamp.sec += 1;
              timesteps -= 10;
            }
            int nsecs = ps.header.stamp.nsec + timesteps*100000000;
            while (nsecs > 1000000000) {
              nsecs -= 1000000000;
              ps.header.stamp.sec += 1;
            }
            ps.header.stamp.nsec = nsecs;
          }

          ps.point.z = current_angle - (steps-j)*ang_vel/10;
          while (ps.point.z < -M_PI) ps.point.z += 2*M_PI;
          while (ps.point.z > M_PI) ps.point.z -= 2*M_PI;
          temp.propagate(ps);
        }
      }
      
      // Propagate the edge for the EKF simulating that transition
      // Find the edge that connects current node to previous node so we can simulate 
      if (forward) {
        for (int j = 0; j < traversing_edge.line.size(); j++) {
          temp.propagate(traversing_edge.line.at(j));
        }
      }
      else {
        for (int j = traversing_edge.line.size()-1; j >= 0; j--) {
          // Have to flip the orientation of the robot as if it had traversed it from other direction
          geometry_msgs::PointStamped ps = traversing_edge.line.at(j);
          double reversed_angle = ps.point.z + M_PI;
          if (reversed_angle > M_PI) reversed_angle -= 2*M_PI;
          ps.point.z = reversed_angle;
          temp.propagate(ps);
        }
      }
      
      geometry_msgs::Point32 pl;
      pl.x = temp.X(3*next_node + 3);
      pl.y = temp.X(3*next_node + 4);
      geometry_msgs::Point32 measurement;
      measurement.x = this->laser_offset_x;
      measurement.y = this->laser_offset_y;
      temp.update(next_node, pl, measurement);
      
      // 2 cases: not in open_set or in open set. If in open set, need to check if current path to that node is quicker than previous path
      // Find if it is in open set
      bool within_open = true;
      if (forward) {
        std::set<int>::iterator it;
        if (open_set.empty()) {
          within_open = false;
        }
        else {
          it = open_set.find(current_edges.at(i).target);
          if (it == open_set.end())
            within_open = false;
        }
      }
      else {
        std::set<int>::iterator it;
        if (open_set.empty()) {
          within_open = false;
        }
        else {
          it = open_set.find(current_edges.at(i).source);
          if (it == open_set.end())
            within_open = false;
        }
      }
      // already in open set, so look through paths to compare new path and old path
      if (within_open) {
        int index = 0;
        for (int j = 0; j < paths.size(); j++) {
          if (paths.at(j).current_node == next_node) {
            index = j;
            break;
          }
        }
        
        temp.total_distance += traversing_edge.length;
        
        // Remove old path because it has higher cost than new path to the same node
        if (temp.computeCost(target) < paths.at(index).computeCost(target)) {
          paths.erase(paths.begin() + index);
          came_from[next_node] = current.current_node;
          temp.last_node = current.current_node;
          temp.current_node = next_node;
          paths.push_back(temp);
        }
      }
      else {
        if (forward) {
          came_from[next_node] = current.current_node;
          temp.last_node = current.current_node;
          temp.current_node = next_node;
          open_set.insert(next_node);
          temp.total_distance += traversing_edge.length;
        } else {
          came_from[next_node] = current.current_node;
          temp.last_node = current.current_node;
          temp.current_node = next_node;
          open_set.insert(next_node);
          temp.total_distance += traversing_edge.length;
        }
        paths.push_back(temp);
      }
    }
  }
}
 
std::vector<int> Relocalization_Simulation::backtrace(std::map<int, int> map, int source, int target) {
  std::vector<int> result;
  int current = target;
  while (current != source) {
    result.push_back(current);
    current = map[current];
  }
  return result;
}

/*
 * Pass meetpoint/endpoint signatures from Follower to Mapper.
 */

Planner::Planner() {
  this->state = 0;
  this->select_bearing_srv = nh.advertiseService("select_bearing", &Planner::SelectBearingGVG, this);
  this->retrieve_bearings_cln = nh.serviceClient<gvg_mapper::RetrieveBearings>("/retrieve_bearings");
  this->retrieve_path_cln = nh.serviceClient<gvg_mapper::RetrievePath>("/retrieve_path");
  this->check_relocalize_cln = nh.serviceClient<gvg_mapper::CheckRelocalize>("/check_relocalize");
  this->min_uncertainty_cln = nh.serviceClient<localizer::MinUncertainty>("/indoor/gvg/MinUncertainty");
  this->max_uncertainty_cln = nh.serviceClient<localizer::MaxUncertainty>("/indoor/gvg/MaxUncertainty");
  this->map_sub = nh.subscribe("/indoor/gvg/theMap", 1, &Planner::handleMap, this);
  
  frontier_min_uncertainty_exploration = false;
  nh.getParam("/gvg_planner/frontier_min_uncertainty_exploration", frontier_min_uncertainty_exploration);
  nh.getParam("/gvg_planner/random_reloc_target", random_reloc_target);
  nL = 0;
  X=VectorXd(3);
  X << 0,0,0;
  P = MatrixXd::Zero(3,3); // The covariance matrix
  goto_min_frontier = true;
}

void Planner::handleMap(const localizer::GVGmap& map) {
  if ((unsigned) X.size() < map.state.size()) {
    X.conservativeResize(map.state.size());
  }
  nL = (map.state.size()-3)/3;
  int i = 0;
  for(std::vector<double>::const_iterator it = map.state.begin();it != map.state.end(); ++it) {
    X(i) = *it;
    i++;
  }
  std::vector<double>::const_iterator it = map.cov.begin();
  unsigned int covSize = 3*nL+3;
  if(P.cols() != covSize) {
    P.conservativeResize(covSize,covSize);
  }
  for(unsigned int i = 0; i < covSize; i++) {
    for(unsigned int j = 0; j < covSize; j++) {
      P(i,j) = *it;
      it++;
    }
  }
}

/*
 * Query the Mapper for information about current node and retrieve selected bearing for next exploration.
 * If the meetpoint has all visited edges, retrieve path plan from Mapper to return to previous meetpoint with unexplored edges.
 */
bool Planner::SelectBearingGVG(gvg_planner::SelectBearing::Request& req, gvg_planner::SelectBearing::Response& res) {
        
  // Exploration state
  if (state == 0) {
    
    /* Check relocalize */
    gvg_mapper::CheckRelocalize relocalize_srv;
    if (!check_relocalize_cln.call(relocalize_srv)) {
      ROS_WARN("Could not check relocalize from Mapper!");
      res.success = false;
      return true;
    }
    
    
    if (frontier_min_uncertainty_exploration && goto_min_frontier && nL > 1) {
      goto_min_frontier = false;
      // Before running retrieve Bearings and checking which direction to go next from current node, run heuristic search
      // and find the node that will most reduce uncertainty then continue exploration
      double Wvv = 0.0;
      double Www = 0.0;
      double Wvw = 0.0;
      nh.getParam("/indoor/gvg/localizerGVGNode/Wvv", Wvv);
      nh.getParam("/indoor/gvg/localizerGVGNode/Www", Www);
      nh.getParam("/indoor/gvg/localizerGVGNode/Wvw", Wvw);
      
      double laser_offset_x = 0.0;
      double laser_offset_y = 0.0;
      nh.getParam("/gvg_mapper/laser_distance_x", laser_offset_x);
      nh.getParam("/gvg_mapper/laser_distance_y", laser_offset_y);
      
      ROS_WARN("Current trace uncertainty: %f", P.block(3, 3, 3*nL, 3*nL).trace());
      
      Relocalization_Simulation reloc_sim(laser_offset_x, laser_offset_y, Wvv, Wvw, Www, nL, X, P);
      
      // Run this code to print out the heuristic search output to all nodes in map
      
      // Use relocalization sim to find path to min uncertainty node
      double min_uncertainty = INFINITY;
      
      double alpha = 0.0;
      nh.getParam("/gvg_planner/Astar_weight", alpha);
      
      for (int j = 0; j < nL; j++) {
        if (req.node_id != j) {
          
          double start_shortest_distance = INFINITY;
          gvg_mapper::RetrievePath pathsrv;
          pathsrv.request.source = req.node_id;
          pathsrv.request.target = j;
          if (!retrieve_path_cln.call(pathsrv)) {
            ROS_WARN("Could not retrieve shortest distance to target from Mapper!");
          }
          else {
            start_shortest_distance = pathsrv.response.distance;
          }
          if (!pathsrv.response.frontier) {
            continue;
          }
          ROS_INFO("Running heuristic search from %d to %d", req.node_id, j);
          double uncertainty = 0.0;
          vector<int> temp_path = reloc_sim.find_optimal(req.node_id, j, start_shortest_distance, alpha, uncertainty);
          if (uncertainty < min_uncertainty) {
            vertex_list.assign(temp_path.begin(), temp_path.end());
            min_uncertainty = uncertainty;
          }
        }
      }
      
      ROS_WARN("Final min uncertainty: %f", min_uncertainty);
      cout << "Output path from heuristic search: ";
      for (int i = vertex_list.size() - 1; i >= 0; i--) {
        if (i != 0) std::cout << vertex_list.at(i) << " -> ";
        else std::cout << vertex_list.at(i) << std::endl;
      }
      
      if (vertex_list.size() > 0) {
        this->state = 1;
          
        target = vertex_list.back();
        vertex_list.pop_back();

        // Retrieve the bearing we need to head next.
        gvg_mapper::RetrieveBearings srv;
        srv.request.node_id = req.node_id;
        srv.request.target = target;
        if (!retrieve_bearings_cln.call(srv)) {
          ROS_WARN("Could not retrieve bearings from Mapper!");
          res.success = false;
          return true;
        }
        double chosen_bearing = srv.response.chosen_bearing;

        ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
        res.selected_bearing = chosen_bearing;

        res.success = true; 
        if (vertex_list.size() == 0) this->state = 0;
        return true;
      }
    }
    
    goto_min_frontier = true;
    
    
    
    /* Retrieve available bearings from gvg_mapper */
    gvg_mapper::RetrieveBearings srv;
    srv.request.node_id = req.node_id;
    srv.request.target = -1;
    if (!retrieve_bearings_cln.call(srv)) {
      ROS_WARN("Could not retrieve bearings from Mapper!");
      res.success = false;
      return true;
    }
    
    // Relocalize, pathplan to node with MIN uncertainty
    if (relocalize_srv.response.relocalize == 1) {      
      ROS_INFO("Robot uncertainty too large. Path planning to minimum uncertainty vertex.");
      localizer::MinUncertainty minsrv;
      if (!min_uncertainty_cln.call(minsrv)) {
        ROS_WARN("Could not get minimum uncertainty vertex from localizer!");
        res.success = false;
        return true;
      }
      gvg_mapper::RetrievePath pathsrv;
      pathsrv.request.source = req.node_id;
      pathsrv.request.target = minsrv.response.id;
      if (!retrieve_path_cln.call(pathsrv)) {
        ROS_WARN("Could not retrieve path plan from Mapper!");
        res.success = false;
        return true;
      }
      
      vertex_list = pathsrv.response.vertex_list;
      this->state = 1;
      
      target = vertex_list.back();
      vertex_list.pop_back();

      // Retrieve the bearing we need to head next.
      gvg_mapper::RetrieveBearings srv;
      srv.request.node_id = req.node_id;
      srv.request.target = target;
      if (!retrieve_bearings_cln.call(srv)) {
        ROS_WARN("Could not retrieve bearings from Mapper!");
        res.success = false;
        return true;
      }
      double chosen_bearing = srv.response.chosen_bearing;
      
      ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
      res.selected_bearing = chosen_bearing;

      res.success = true; 
      if (vertex_list.size() == 0) this->state = 0;
      return true;
    }
    
    // Random relocalization, path plan to MIN uncertainty vertex
    if (relocalize_srv.response.relocalize == 2) {
      ROS_INFO("Random relocalization. Path planning to minimum uncertainty vertex.");
      
      // The model noise covariance (linear and angular velocity) defined in localizer launch files
      double Wvv = 0.0;
      double Www = 0.0;
      double Wvw = 0.0;
      nh.getParam("/indoor/gvg/localizerGVGNode/Wvv", Wvv);
      nh.getParam("/indoor/gvg/localizerGVGNode/Www", Www);
      nh.getParam("/indoor/gvg/localizerGVGNode/Wvw", Wvw);
      
      double laser_offset_x = 0.0;
      double laser_offset_y = 0.0;
      nh.getParam("/gvg_mapper/laser_distance_x", laser_offset_x);
      nh.getParam("/gvg_mapper/laser_distance_y", laser_offset_y);
      
      
      Relocalization_Simulation reloc_sim(laser_offset_x, laser_offset_y, Wvv, Wvw, Www, nL, X, P);
      
      // Run this code to print out the heuristic search output to all nodes in map
      
      // Use relocalization sim to find path to min uncertainty node
      double min_uncertainty = INFINITY;
      
      double alpha = 0.0;
      nh.getParam("/gvg_planner/Astar_weight", alpha);
      
      for (int j = 0; j < nL; j++) {
        if (req.node_id != j) {
          
          double start_shortest_distance = INFINITY;
          gvg_mapper::RetrievePath pathsrv;
          pathsrv.request.source = req.node_id;
          pathsrv.request.target = j;
          if (!retrieve_path_cln.call(pathsrv)) {
            ROS_WARN("Could not retrieve shortest distance to target from Mapper!");
          }
          else {
            start_shortest_distance = pathsrv.response.distance;
          }
          
          ROS_INFO("Running heuristic search from %d to %d", req.node_id, j);
          double uncertainty = 0.0;
          vector<int> temp_path = reloc_sim.find_optimal(req.node_id, j, start_shortest_distance, alpha, uncertainty);
          if (uncertainty < min_uncertainty) {
            vertex_list.assign(temp_path.begin(), temp_path.end());
            min_uncertainty = uncertainty;
          }
        }
      }
      
      ROS_WARN("Final min uncertainty: %f", min_uncertainty);
      cout << "Output path from heuristic search: ";
      for (int i = vertex_list.size() - 1; i >= 0; i--) {
        if (i != 0) std::cout << vertex_list.at(i) << " -> ";
        else std::cout << vertex_list.at(i) << std::endl;
      }
      
      
      /*
      localizer::MinUncertainty minsrv;
      if (!min_uncertainty_cln.call(minsrv)) {
        ROS_WARN("Could not get minimum uncertainty vertex from localizer!");
        res.success = false;
        return true;
      }
      
      gvg_mapper::RetrievePath pathsrv;
      pathsrv.request.source = req.node_id;
      pathsrv.request.target = minsrv.response.id;
      if (!retrieve_path_cln.call(pathsrv)) {
        ROS_WARN("Could not retrieve path plan from Mapper!");
        res.success = false;
        return true;
      }
      
      vertex_list = pathsrv.response.vertex_list;*/
      this->state = 1;
      
      target = vertex_list.back();
      vertex_list.pop_back();

      // Retrieve the bearing we need to head next.
      gvg_mapper::RetrieveBearings srv;
      srv.request.node_id = req.node_id;
      srv.request.target = target;
      if (!retrieve_bearings_cln.call(srv)) {
        ROS_WARN("Could not retrieve bearings from Mapper!");
        res.success = false;
        return true;
      }
      double chosen_bearing = srv.response.chosen_bearing;

      ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
      res.selected_bearing = chosen_bearing;

      res.success = true; 
      if (vertex_list.size() == 0) this->state = 0;
      return true;
    }

    double chosen_bearing = srv.response.chosen_bearing;

    // Still unexplored edges to visit
    if (chosen_bearing != NULL) {
      ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
      res.selected_bearing = chosen_bearing;

      res.success = true; 
      return true;
    }
    // Explored all edges at this vertex, path plan to previously nonexplored edges vertex.
    else {
      ROS_INFO("All edges explored at this vertex. Requesting path to previously non-explored edges vertex.");
      gvg_mapper::RetrievePath pathsrv;
      pathsrv.request.source = req.node_id;
      pathsrv.request.target = -1;
      if (!retrieve_path_cln.call(pathsrv)) {
        ROS_WARN("Could not retrieve path plan from Mapper!");
        res.success = false;
        return true;
      }

      // Relocalization
      if ((int) pathsrv.response.vertex_list.size() == 0) {
        /*if (!repositioned) {
          repositioned = true;
          ROS_WARN("Where to reposition to?");
          string str = "";
          getline(cin, str);
          stringstream stream(str);
          int target;
          stream >> target;        
          
          pathsrv.request.source = req.node_id;
          pathsrv.request.target = target;
          if (!retrieve_path_cln.call(pathsrv)) {
            ROS_WARN("Could not retrieve path plan from Mapper!");
            res.success = false;
            return true;
          }
          vertex_list = pathsrv.response.vertex_list;
          ROS_INFO("Path planning to return to vertex %d", vertex_list[0]);
          std::cout << "Shortest path proposed: ";
          for (int i = vertex_list.size() - 1; i >= 0; i--) {
            if (i != 0) std::cout << vertex_list.at(i) << " -> ";
            else std::cout << vertex_list.at(i) << std::endl;
          }
        }*/
        ROS_WARN("Entire map has been explored. Relocalizing...");
        ROS_WARN("Current trace of covariance: %.2f", P.block(3, 3, 3*nL, 3*nL).trace());
        
        int target_node;
        
        if (random_reloc_target) {
          target_node = req.node_id;
          while (target_node == req.node_id) {
            target_node = rand() % nL;
          }
        }
        else {
          localizer::MaxUncertainty maxsrv;
          if (!max_uncertainty_cln.call(maxsrv)) {
            ROS_WARN("Could not get maximum uncertainty vertex from localizer!");
            res.success = false;
            return true;
          }
          target_node = maxsrv.response.id;
          if (req.node_id == maxsrv.response.id) {
            localizer::MaxUncertainty minsrv;
            if (!min_uncertainty_cln.call(minsrv)) {
              ROS_WARN("Could not get minimum uncertainty vertex from localizer!");
              res.success = false;
              return true;
            }
            target_node = minsrv.response.id;
          }
        }
        
        // The model noise covariance (linear and angular velocity) defined in localizer launch files
        double Wvv = 0.0;
        double Www = 0.0;
        double Wvw = 0.0;
        nh.getParam("/indoor/gvg/localizerGVGNode/Wvv", Wvv);
        nh.getParam("/indoor/gvg/localizerGVGNode/Www", Www);
        nh.getParam("/indoor/gvg/localizerGVGNode/Wvw", Wvw);
        
        double laser_offset_x = 0.0;
        double laser_offset_y = 0.0;
        nh.getParam("/gvg_mapper/laser_distance_x", laser_offset_x);
        nh.getParam("/gvg_mapper/laser_distance_y", laser_offset_y);
        
        
        Relocalization_Simulation reloc_sim(laser_offset_x, laser_offset_y, Wvv, Wvw, Www, nL, X, P);
        
        // Run this code to print out the heuristic search output to all nodes in map
        
        // Use relocalization sim to find path to min uncertainty node
        double min_uncertainty = INFINITY;
        
        double alpha = 0.0;
        nh.getParam("/gvg_planner/Astar_weight", alpha);
        
        for (int j = 0; j < nL; j++) {
          if (req.node_id != j) {
            
            double start_shortest_distance = INFINITY;
            pathsrv.request.source = req.node_id;
            pathsrv.request.target = j;
            if (!retrieve_path_cln.call(pathsrv)) {
              ROS_WARN("Could not retrieve shortest distance to target from Mapper!");
            }
            else {
              start_shortest_distance = pathsrv.response.distance;
            }
            
            ROS_INFO("Running heuristic search from %d to %d", req.node_id, j);
            double uncertainty = 0.0;
            vector<int> temp_path = reloc_sim.find_optimal(req.node_id, j, start_shortest_distance, alpha, uncertainty);
            if (uncertainty < min_uncertainty) {
              vertex_list.assign(temp_path.begin(), temp_path.end());
              min_uncertainty = uncertainty;
            }
          }
        }
        
        ROS_WARN("Final min uncertainty: %f", min_uncertainty);
        cout << "Output path from heuristic search: ";
        for (int i = vertex_list.size() - 1; i >= 0; i--) {
          if (i != 0) std::cout << vertex_list.at(i) << " -> ";
          else std::cout << vertex_list.at(i) << std::endl;
        }
        // Run this code to actually relocalize to max/min uncertainty node
        /*
        double start_shortest_distance = INFINITY;
        pathsrv.request.source = req.node_id;
        pathsrv.request.target = target_node;
        if (!retrieve_path_cln.call(pathsrv)) {
          ROS_WARN("Could not retrieve shortest distance to target from Mapper!");
        }
        else {
          start_shortest_distance = pathsrv.response.distance;
        }
        
        double alpha = 0.0;
        nh.getParam("/gvg_planner/Astar_weight", alpha);
        
        ROS_INFO("Running heuristic search from %d to %d", req.node_id, target_node);
        vertex_list = reloc_sim.find_optimal(req.node_id, target_node, start_shortest_distance, alpha);
        cout << "Output path from heuristic search: ";
        for (int i = vertex_list.size() - 1; i >= 0; i--) {
          if (i != 0) std::cout << vertex_list.at(i) << " -> ";
          else std::cout << vertex_list.at(i) << std::endl;
        }*/
        
        
        /*STOP AFTER A FEW ITERATIONS I SUPPOSE 
         * nh.setParam("/indoor/gvg/agent/do_move", false);
        res.success = true;
        return true;*/
      }
      else {
        vertex_list = pathsrv.response.vertex_list;
        ROS_INFO("Path planning to return to vertex %d", vertex_list[0]);
        std::cout << "Shortest path proposed: ";
        for (int i = vertex_list.size() - 1; i >= 0; i--) {
          if (i != 0) std::cout << vertex_list.at(i) << " -> ";
          else std::cout << vertex_list.at(i) << std::endl;
        }
      }
      this->state = 1;
      
      target = vertex_list.back();
      vertex_list.pop_back();

      // Retrieve the bearing we need to head next.
      gvg_mapper::RetrieveBearings srv;
      srv.request.node_id = req.node_id;
      srv.request.target = target;
      if (!retrieve_bearings_cln.call(srv)) {
        ROS_WARN("Could not retrieve bearings from Mapper!");
        res.success = false;
        return true;
      }
      double chosen_bearing = srv.response.chosen_bearing;

      ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
      res.selected_bearing = chosen_bearing;

      res.success = true; 
      if (vertex_list.size() == 0) this->state = 0;
      return true;
    }
  
  // Pathplanning state
  } else {
    if (target != req.node_id) {
      ROS_WARN("Robot did not reach the planned target. Restarting path planning.");
      gvg_mapper::RetrievePath pathsrv;
      pathsrv.request.source = req.node_id;
      if (!retrieve_path_cln.call(pathsrv)) {
        ROS_WARN("Could not retrieve path plan from Mapper!");
        res.success = false;
        return true;
      }
      vertex_list = pathsrv.response.vertex_list;
      this->state = 1;
      
      target = vertex_list.back();
      vertex_list.pop_back();

      // Retrieve the bearing we need to head next.
      gvg_mapper::RetrieveBearings srv;
      srv.request.node_id = req.node_id;
      srv.request.target = target;
      if (!retrieve_bearings_cln.call(srv)) {
        ROS_WARN("Could not retrieve bearings from Mapper!");
        res.success = false;
        return true;
      }
      double chosen_bearing = srv.response.chosen_bearing;

      ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
      res.selected_bearing = chosen_bearing;

      res.success = true; 
      if (vertex_list.size() == 0) this->state = 0;
      return true;
    }
    target = vertex_list.back();
    vertex_list.pop_back();

    // Retrieve the bearing we need to head next.
    gvg_mapper::RetrieveBearings srv;
    srv.request.node_id = req.node_id;
    srv.request.target = target;
    if (!retrieve_bearings_cln.call(srv)) {
      ROS_WARN("Could not retrieve bearings from Mapper!");
      res.success = false;
      return true;
    }
    double chosen_bearing = srv.response.chosen_bearing;

    ROS_INFO("Requested direction from GVG Follower: %.2f", chosen_bearing * 180.0/M_PI);
    res.selected_bearing = chosen_bearing;

    res.success = true; 
    if (vertex_list.size() == 0) this->state = 0;
    return true;  
  }
}
