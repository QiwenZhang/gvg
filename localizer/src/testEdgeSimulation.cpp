#include "localizerGVG.h"
#include "ros/ros.h"
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include "localizer/GVGmap.h"
#include "gvg_mapper/GVGNode.h"
#include "gvg_mapper/GVGEdgeMsg.h"
#include "gvg_mapper/LoadSavedMap.h"
#include "localizer/LoadLocalizerMap.h"
#include "localizer/UpdateFilter.h"

/* Don't use this code, it doesn't work with the proper version of localizer
 * 
 */

int main(int argc, char **argv) {
    std::string robot_name = "indoor";
    ros::init(argc, argv, robot_name);
    
    ROS_WARN("Testing");
    ros::NodeHandle nh;
    ros::ServiceClient load_localizer_cln;
    load_localizer_cln = nh.serviceClient<localizer::LoadLocalizerMap>("/indoor/gvg/LoadLocalizerMap");
    ros::ServiceClient update_cln;
    update_cln = nh.serviceClient<localizer::UpdateFilter>("/indoor/gvg/UpdateFilter");
  
    ifstream myfile;
    std::vector<gvg_mapper::GVGNode> nodes;
    
    string mapper_nodes = "/home/thalassa/qzhang32/catkin_ws/cross2-nodes";
    myfile.open(mapper_nodes.c_str());
    
    if (myfile.is_open()) {
      char buf[128];
      char* token;
      myfile.getline(buf, 128);
      token = strtok(buf, ",");
      
      int num_nodes = atoi(token);
      for (int i = 0; i < num_nodes; i++) {
        gvg_mapper::GVGNode v;
        // Create vertex and load information from file
        myfile.getline(buf, 128);
        v.node_id = atoi(buf);
        myfile.getline(buf, 128);
        v.p.x = atof(strtok(buf, ","));
        v.p.y = atof(strtok(NULL, ","));
        v.p.z = atof(strtok(NULL, ","));
        myfile.getline(buf, 128);
        v.degree = atoi(buf);
        myfile.getline(buf, 128);
        v.closest_distance = atof(buf);
        myfile.getline(buf, 128);
        v.vertex_angle = atof(buf);
        myfile.getline(buf, 128);
        token = strtok(buf, ",");
        while (token != NULL) {
          v.edge_angle_diffs.push_back(atof(token));
          token = strtok(NULL, ",");
        }
        for (int j = 0; j < v.degree; j++) {
          gvg_mapper::EdgeBearingMsg eb;
          myfile.getline(buf, 128);
          token = strtok(buf, ",");
          eb.bearing_id = atoi(token);
          token = strtok(NULL, ",");
          eb.edge_id = atoi(token);
          token = strtok(NULL, ",");
          eb.edge_angle_diff = atof(token);
          token = strtok(NULL, ",");
          eb.visited = atoi(token);
          v.possible_bearings.push_back(eb);
        }
        nodes.push_back(v);
      }
    }
    else {
      ROS_WARN("Unable to load map nodes!");
    }
    /*for (int i = 0; i < this->nodes.size(); i++) {
      ROS_INFO("%d", this->nodes.at(i).node_id);
      for (int j = 0; j < this->nodes.at(i).possible_bearings.size(); j++) {
        ROS_WARN("%d %d %f %d", this->nodes.at(i).possible_bearings.at(j).bearing_id, this->nodes.at(i).possible_bearings.at(j).edge_id, this->nodes.at(i).possible_bearings.at(j).edge_angle_diff, this->nodes.at(i).possible_bearings.at(j).visited);
      }
    }*/
    myfile.close();
    
    //////////////////////////////////////
    
    string localizer_map = "/home/thalassa/qzhang32/catkin_ws/cross2-state";
    myfile.open(localizer_map.c_str());
    
    VectorXd X; // The State Vector
    MatrixXd P; // The Covariance Matrix
    int nL = 0;
    
    if (myfile.is_open()) {
      
      char buf[4096];
      
      char* token;
      myfile.getline(buf, 4096);
      token = strtok(buf,",");
      
      nL = atoi(token);
      //ROS_WARN("%d nL", nL);
      X = VectorXd(nL*3 + 3);
      P= MatrixXd::Zero(nL*3 + 3,nL*3 + 3);
            
      // Save the X matrix      
      myfile.getline(buf, 4096);
      token = strtok(buf,",");
      int i = 0;
      while (token != NULL) {
        X(i) = atof(token);
        token = strtok(NULL, ",");
          //cout << X(i);
          //cout << " ";
        i++;
      }
      //cout << "\n";
            
      int j = 0;
      while ( myfile.getline(buf, 4096)) {
        // Save the P matrix
        i = 0;
        token = strtok(buf,",");
        while (token != NULL)
        {
          P(i,j) = atof(token);
          token = strtok(NULL, ",");
          //cout << P(i,j);
          //cout << " ";
          i++;
        }
        //cout << "\n";
        j++;
      }
    }
    else {
      ROS_WARN("Unable to load localizer map!");
    }
    myfile.close();
    localizer::LoadLocalizerMap srv;
    
    for (int i = 0; i < nodes.size(); i++) {
      srv.request.ids.push_back(nodes.at(i).node_id);
    }
      
    srv.request.nL = nL;    
    for(unsigned int i = 0; i < X.size(); i++) {
      srv.request.state.push_back(X(i));
    }
    for(unsigned int i=0; i<P.rows(); i++) {
      for(unsigned int j=0; j<P.cols(); j++) {
        srv.request.cov.push_back(P(i,j));
      }
    }
    
    if (!load_localizer_cln.call(srv)) {
      ROS_WARN("Could not call load localizer service!");
    }
    
    //////////////////////////
    
    std::vector<gvg_mapper::GVGEdgeMsg> edges;
    string mapper_edges = "/home/thalassa/qzhang32/catkin_ws/cross2-edges";
    myfile.open(mapper_edges.c_str());
    
    if (myfile.is_open()) {
      char buf[128];
      char* token;
      while ( myfile.getline(buf, 128)) {
        gvg_mapper::GVGEdgeMsg e;
        token = strtok(buf, ",");
        e.edge_id = atoi(token);
        token = strtok(NULL, ",");
        e.source = atoi(token);
        token = strtok(NULL, ",");
        e.target = atoi(token);
        token = strtok(NULL, ",");
        e.length = atof(token);
        myfile.getline(buf, 128);
        while (myfile.getline(buf, 128)) {
          if (strcmp("---", buf) == 0) {
            //ROS_WARN("BREAKING");
            break;
          }
          else {
            geometry_msgs::PointStamped p;
            token = strtok(buf, ",");
            double num = atof(token);
            int num_int = floor(num);
            p.header.stamp.sec = num;
            p.header.stamp.nsec = (int) ((num - (double) num_int)*1000000000);
            token = strtok(NULL, ",");
            p.point.x = atof(token);
            token = strtok(NULL, ",");
            p.point.y = atof(token);
            token = strtok(NULL, ",");
            p.point.z = atof(token);
            e.line.push_back(p);
            //ROS_WARN("%f, %f, %f", p.x, p.y, p.z);
          }
        }
        edges.push_back(e);
      }
    }
    /*for (int i = 0; i < this->edges.size(); i++) {
      ROS_INFO("%d %d %d %f", this->edges.at(i).edge_id, this->edges.at(i).source, this->edges.at(i).target, this->edges.at(i).length);
    }*/
    myfile.close();
    
    ros::Publisher posePub = nh.advertise<nav_msgs::Odometry>("/indoor/odom", 10);
    for (int i = 0; i < edges.size(); i++) {
      if (edges.at(i).target == 4) {
        for (int j = edges.at(i).line.size()-1; j > -1; j--) {
          nav_msgs::Odometry msg;
          msg.pose.pose.position.x = edges.at(i).line.at(j).point.x;
          msg.pose.pose.position.y = edges.at(i).line.at(j).point.y;
          msg.pose.pose.position.z = edges.at(i).line.at(j).point.z + M_PI;
          if (msg.pose.pose.position.z > M_PI) {
            msg.pose.pose.position.z -= 2*M_PI;
          }
          msg.header.stamp = edges.at(i).line.at(j).header.stamp;
          posePub.publish(msg);
        }
        ros::Rate r(1);
        r.sleep();
      }
    }
    
    localizer::UpdateFilter uf;
    uf.request.id = 0;
    uf.request.x = 0.225;
    uf.request.y = 0.0;
    uf.request.yaw = M_PI;
    
    if (!update_cln.call(uf)) {
      ROS_WARN("Could not call load localizer service!");
    }
    
    ROS_WARN("Done testing");
}
