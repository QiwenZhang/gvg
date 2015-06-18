#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <gvg_mapper/GVGNode.h>
#include <gvg_mapper/GVGEdgeMsg.h>
#include <nav_msgs/Odometry.h>
#include <boost/lexical_cast.hpp>

int id;
std::string frame_id;
ros::Publisher meetpoints_pub;
ros::Publisher meetpoints_names_pub;
ros::Publisher edges_pub;
ros::Publisher edges_names_pub;
ros::Publisher odom_pub;
void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg);
void handle_nodes(const gvg_mapper::GVGNode::ConstPtr& msg);
void handle_edges(const gvg_mapper::GVGEdgeMsg::ConstPtr& msg);

int main( int argc, char** argv ) {
  ros::init(argc, argv, "gvg_mapper_viz");
  ros::NodeHandle nh;
  meetpoints_pub = nh.advertise<visualization_msgs::Marker>("meetpoints", 1);
  meetpoints_names_pub = nh.advertise<visualization_msgs::Marker>("meetpoints_names", 1);
  edges_pub = nh.advertise<visualization_msgs::Marker>("edges", 1);
  edges_names_pub = nh.advertise<visualization_msgs::Marker>("edges_names", 1);
  odom_pub = nh.advertise<visualization_msgs::Marker>("robot_path", 1);
  ros::Subscriber node_sub = nh.subscribe("/node", 1, &handle_nodes);
  ros::Subscriber edge_sub = nh.subscribe("/edge", 1, &handle_edges);
  ros::Subscriber odom_sub = nh.subscribe("/indoor/gvg/odom_combined", 1, &handle_odometry);

  frame_id = argv[1];
  
  ros::spin();
  return 0;
}

void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg) {
  std::string ns = "path";
  
  // Populating robot path markers
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = msg->header.stamp;
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1; m.scale.y = 0.1;   
  m.lifetime = ros::Duration(0);

  m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 0.0;  m.color.b = 1.0;
  geometry_msgs::Point p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  m.points.push_back(p);
  
  odom_pub.publish(m);
}

void handle_nodes(const gvg_mapper::GVGNode::ConstPtr& msg) {
  std::string ns = "gvg_mapper_nodes";

  // Populating node markers
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = msg->header.stamp;
  m.ns = ns;
  m.id = msg->node_id;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1; m.scale.y = 0.1;   
  m.lifetime = ros::Duration(0);

  m.color.a = 1.0;   m.color.r = 1.0;   m.color.g = 0.0;  m.color.b = 0.0;
  geometry_msgs::Point p;
  p.x = msg->p.x;
  p.y = msg->p.y;
  m.points.push_back(p);

  // Populating node ids 
  visualization_msgs::Marker m_id;
  m_id.header.frame_id = frame_id;
  m_id.header.stamp = msg->header.stamp;
  m_id.ns = ns;
  m_id.id = msg->node_id;
  m_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m_id.action = visualization_msgs::Marker::ADD;
  m_id.scale.z = 0.6;
  m_id.lifetime = ros::Duration(0);

  m_id.color.a = 1.0; m_id.color.r = 1.0; m_id.color.g = 0.0; m_id.color.b = 0.0;
  m_id.text = boost::lexical_cast<std::string>(msg->node_id);
  m_id.pose.position.x = msg->p.x;
  m_id.pose.position.y = msg->p.y;
  m_id.pose.position.z = msg->p.z + 0.4;

  meetpoints_names_pub.publish(m_id);
  meetpoints_pub.publish(m);
}

void handle_edges(const gvg_mapper::GVGEdgeMsg::ConstPtr& msg) {
  std::string ns = "gvg_mapper_edges";

  // Populating edges
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = msg->header.stamp;
  m.ns = ns;
  m.id = msg->edge_id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.06;
  m.lifetime = ros::Duration(0);
    
  m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 1.0;  m.color.b = 0.0;

  std::vector<geometry_msgs::Point> pts;
  for (int j = 0; j < (int) msg->line.size(); j++) {
    geometry_msgs::Point pt;
    pt.x = msg->line.at(j).point.x;
    pt.y = msg->line.at(j).point.y;
    pts.push_back(pt);
  }
  
  m.points.assign(pts.begin(), pts.end());

  // Populating edge ids 
  visualization_msgs::Marker m_id;
  m_id.header.frame_id = frame_id;
  m_id.header.stamp = msg->header.stamp;
  m_id.ns = ns;
  m_id.id = msg->edge_id;
  m_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m_id.action = visualization_msgs::Marker::ADD;
  m_id.scale.z = 0.6;
  m_id.lifetime = ros::Duration(0);

  m_id.color.a = 1.0; m_id.color.r = 0.0; m_id.color.g = 1.0; m_id.color.b = 0.0;
  m_id.text = boost::lexical_cast<std::string>(msg->edge_id);
  int size = msg->line.size();
  m_id.pose.position.x = msg->line.at(floor(size/2.0)).point.x;
  m_id.pose.position.y = msg->line.at(floor(size/2.0)).point.y;
  m_id.pose.position.z = 0.4;

  edges_names_pub.publish(m_id);  
  edges_pub.publish(m);
}
