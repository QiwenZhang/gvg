#include "ros/ros.h"
#include "localizer/GVGmap.h"
#include "gvg_mapper/GVGNode.h"
#include "gvg_mapper/GVGEdgeMsg.h"
#include "gvg_mapper/LoadSavedMap.h"
#include "localizer/LoadLocalizerMap.h"
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

class MapHandler {
public:
  
	MapHandler(ros::NodeHandle& nh) { 
    map_sub = nh.subscribe("/indoor/gvg/theMap", 1, &MapHandler::handleMap, this);
    node_sub = nh.subscribe("/node", 1, &MapHandler::handleNode, this);
    edge_sub = nh.subscribe("/edge", 1, &MapHandler::handleEdge, this);
    this->load_mapper_cln = nh.serviceClient<gvg_mapper::LoadSavedMap>("/load_saved_map");
    this->load_localizer_cln = nh.serviceClient<localizer::LoadLocalizerMap>("/indoor/gvg/LoadLocalizerMap");
    this->save_filename = "";
    this->load_filename = "";
    this->load = false;
    
    nh.getParam("/indoor/gvg/map_handler/map_load_filename", this->load_filename);
    nh.getParam("/indoor/gvg/map_handler/load", this->load);
    if (this->load) {
      loadMapper();
      loadLocalizer();
    }
    
    nh.getParam("/indoor/gvg/map_handler/map_save_filename", this->save_filename);
    // Create each file once
    ofstream myfile;
    string mapper_edges = this->save_filename + "-edges";
    myfile.open(mapper_edges.c_str());
    myfile.close();
    string mapper_nodes = this->save_filename + "-nodes";
    myfile.open(mapper_nodes.c_str());
    myfile.close();
    string localizer_map = this->save_filename + "-state";
    myfile.open(localizer_map.c_str());
    myfile.close();
	}
	
	void spin() {
		while(ros::ok()){
			ros::spinOnce();
		}
		ros::shutdown();
	}

  void loadMapper() {
    ifstream myfile;
    string mapper_nodes = this->load_filename + "-nodes";
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
        this->nodes.push_back(v);
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
    
    
    // Load edges now
    string mapper_edges = this->load_filename + "-edges";
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
        //ROS_WARN("IT WORKS");
        this->edges.push_back(e);
      }
    }
    /*for (int i = 0; i < this->edges.size(); i++) {
      ROS_INFO("%d %d %d %f", this->edges.at(i).edge_id, this->edges.at(i).source, this->edges.at(i).target, this->edges.at(i).length);
    }*/
    myfile.close();
    
    // Send srv to mapper to transmit all data
    
    gvg_mapper::LoadSavedMap srv;
    srv.request.nodes = nodes;
    srv.request.edges = edges;
    
    if (!load_mapper_cln.call(srv)) {
      ROS_WARN("Could not call load gvg_mapper service!");
    }
  }
  
  void loadLocalizer() {
    ifstream myfile;
    string localizer_map = this->load_filename + "-state";
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
    
  }
	
  void handleNode(const gvg_mapper::GVGNode node) {
    ofstream myfile;
    string mapper_nodes = this->save_filename + "-nodes";
    myfile.open(mapper_nodes.c_str());

    // Check if we don't have the node already
    bool exists = false;
    int index = 0;
    for (int i = 0; i < this->nodes.size(); i++) {
      if (this->nodes.at(i).node_id == node.node_id) {
        exists = true;
        index = i;
        break;
      }
    }
    
    // Already exists, update the information
    if (exists) {
      /*geometry_msgs::Point32 p;
      p.x = node.p.x;
      p.y = node.p.y;
      this->nodes.at(index).p = p;*/
      this->nodes.at(index) = node;
    }
    // Doesn't exist, add it to our list
    else this->nodes.push_back(node);
    
    // Print the save file
    myfile << this->nodes.size() << "\n";
    for (int i = 0; i < this->nodes.size(); i++) {
      myfile << this->nodes.at(i).node_id << "\n";
      myfile << this->nodes.at(i).p.x << "," << this->nodes.at(i).p.y << "," << this->nodes.at(i).p.z << "\n";
      myfile << this->nodes.at(i).degree << "\n";
      myfile << this->nodes.at(i).closest_distance << "\n";
      myfile << this->nodes.at(i).vertex_angle << "\n";
      for (int j = 0; j < this->nodes.at(i).edge_angle_diffs.size(); j++) {
        myfile << this->nodes.at(i).edge_angle_diffs.at(j) << ",";
      }
      myfile << "\n";
      for (int j = 0; j < this->nodes.at(i).possible_bearings.size(); j++) {
        myfile << this->nodes.at(i).possible_bearings.at(j).bearing_id << "," << this->nodes.at(i).possible_bearings.at(j).edge_id << ",";
        myfile << this->nodes.at(i).possible_bearings.at(j).edge_angle_diff << "," << this->nodes.at(i).possible_bearings.at(j).visited << "\n";
      }
    }
    myfile.close();
  }
  
  void handleEdge(const gvg_mapper::GVGEdgeMsg edge) {
        
    ofstream myfile;
    string mapper_edges = this->save_filename + "-edges";
    myfile.open(mapper_edges.c_str());

    this->edges.push_back(edge);
    for (int i = 0; i < this->edges.size(); i++) {
      myfile << this->edges.at(i).edge_id << ",";
      myfile << this->edges.at(i).source << ",";
      myfile << this->edges.at(i).target << ",";
      myfile << this->edges.at(i).length << "\n";
      
      // Add each point on the line of the edge
      for (int j = 0; j < this->edges.at(i).line.size(); j++) {
        myfile << this->edges.at(i).line.at(j).header.stamp.toSec() << ",";
        myfile << this->edges.at(i).line.at(j).point.x << ",";
        myfile << this->edges.at(i).line.at(j).point.y << ",";
        myfile << this->edges.at(i).line.at(j).point.z << "\n";
      }
      myfile << "---\n";
    }
    myfile.close();
  }
  
  void handleMap(const localizer::GVGmap& map) {
    
    ofstream myfile;
    string localizer_map = this->save_filename + "-state";
    myfile.open(localizer_map.c_str());

    int nL=(map.state.size()-3)/3;
    myfile << nL;
    myfile << "\n";
    for(std::vector<double>::const_iterator it = map.state.begin(); it != map.state.end(); it++) {
      myfile << *it << ",";
    }
    myfile << "\n";
    std::vector<double>::const_iterator it = map.cov.begin();
    unsigned int covSize = 3*nL+3;

    for(unsigned int i = 0; i < covSize; i++) {
      for(unsigned int j = 0; j < covSize; j++) {
        myfile << *it;
        it++;
        if (j != covSize-1) myfile << ",";
      }
      if (i != covSize-1) myfile << "\n";
    }
    myfile.close();
  }
  
private:
  string save_filename;
  string load_filename;
  bool load;
  std::vector<gvg_mapper::GVGNode> nodes;
  std::vector<gvg_mapper::GVGEdgeMsg> edges;
  ros::Subscriber node_sub;
  ros::Subscriber edge_sub;
	ros::Subscriber map_sub;
  ros::ServiceClient load_mapper_cln;
  ros::ServiceClient load_localizer_cln;
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "map_handler");
	ros::NodeHandle nh;
	MapHandler map_handler(nh);
	map_handler.spin();
  return 0;
}
