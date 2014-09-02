#include <gvg_planner/SelectBearing.h>
#include "Planner.h"
#include <gvg_mapper/RetrieveBearings.h>
#include <gvg_mapper/RetrievePath.h>
#include <gvg_mapper/CheckRelocalize.h>
#include <localizer/MinUncertainty.h>
#include <localizer/MaxUncertainty.h>

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

      // Stop the robot
      if ((int) pathsrv.response.vertex_list.size() == 0) {
        ROS_WARN("Entire map has been explored.");
        nh.setParam("/indoor/gvg/agent/do_move", false);
        res.success = true;
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
