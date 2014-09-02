#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_node/Obstacle.h>
#include <laser_node/Obstacles.h>
#include <gvg/FollowEdgeAction.h>
#include <boost/lexical_cast.hpp>
#include <cmath>

ros::Publisher obstacles_pub;
ros::Publisher obstacles_minima_pub;
ros::Publisher obstacle_names_pub;
ros::Publisher gvg_line_pub;

void handle_follow_edge_feedback(const gvg::FollowEdgeActionFeedbackConstPtr& fbk);

int main( int argc, char** argv ) {
  ros::init(argc, argv, "follow_edge_viz");
  ros::NodeHandle nh;
  obstacle_names_pub   = nh.advertise<visualization_msgs::MarkerArray>("obstacle_names", 1);
  obstacles_pub        = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
  obstacles_minima_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_minima", 1);
  gvg_line_pub         = nh.advertise<visualization_msgs::Marker>("gvg_line", 1);
  ros::Subscriber fef_sub = nh.subscribe("/indoor/gvg/follow_edge/feedback", 1, &handle_follow_edge_feedback);
  ros::spin();
  return 0;
}
 
void handle_follow_edge_feedback(const gvg::FollowEdgeActionFeedbackConstPtr& fbk) {
  gvg::FollowEdgeFeedback msg = fbk->feedback;
  std::string ns = "follow_edge_inputs";
  std::string frame_id = "/base_laser_link";
  
  laser_node::Obstacles obstacles;
  obstacles.collection.push_back(msg.left);
  obstacles.collection.push_back(msg.right);
  
  //ROS_INFO("dy %f", msg.dy);
  //ROS_INFO("dtheta_deg %f", msg.dtheta_in_deg);

  /*
   * Populating obstacles
   */
  visualization_msgs::MarkerArray ma;
  for (int i = 0; i < (int) obstacles.collection.size(); i++) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = i;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.03;  
    
    if (i % 2 == 0) {
      m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 1.0;  m.color.b = 0.0;
    } else {
      m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 0.0;  m.color.b = 1.0;
    }

    std::vector<geometry_msgs::Point> pts;
    for (int j = 0; j < (int) obstacles.collection.at(i).surface.size(); j++) {
      geometry_msgs::Point pt;
      pt.x = obstacles.collection.at(i).surface.at(j).x;
      pt.y = obstacles.collection.at(i).surface.at(j).y;
      pts.push_back(pt);
    }
    
    m.points.assign(pts.begin(), pts.end());
    m.lifetime = ros::Duration(1.0/15.0);   //1/15th of a second
    ma.markers.push_back(m);
  }
  
  /*
   * Populating lines from the robot to the closest points of obstacles
   */
  visualization_msgs::MarkerArray ma_closest;
  for (int i = 0; i < (int) obstacles.collection.size(); i++) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = i + (int) obstacles.collection.size();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.01;  
    
    if (i % 2 == 0) {
      m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 1.0;  m.color.b = 0.0;
    } else {
      m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 0.0;  m.color.b = 1.0;
    }

    geometry_msgs::Point origin;
    origin.x = 0; origin.y = 0;
    
    geometry_msgs::Point closest_pt;
    closest_pt.x = obstacles.collection.at(i).closest_point.x;
    closest_pt.y = obstacles.collection.at(i).closest_point.y;
    m.points.push_back(origin);
    m.points.push_back(closest_pt);
    m.lifetime = ros::Duration(1.0/15.0);   //1/15th of a second
    ma_closest.markers.push_back(m);
  }

  /*
   * Populating names of obstacles
   */
  visualization_msgs::MarkerArray ma_names;
  for (int i = 0; i < (int) obstacles.collection.size(); i++) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ma.markers.at(i).header.stamp;
    m.ns = ns;
    m.id = i + (int) 2*obstacles.collection.size();
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.z = 0.4;  
    m.text = (i == 0) ? "left" : "right";
    m.lifetime = ma.markers.at(i).lifetime;
    m.color = ma.markers.at(i).color;
    int size = (int) obstacles.collection.at(i).surface.size();
    m.pose.position.x = obstacles.collection.at(i).surface.at(floor(size/2.0)).x;
    m.pose.position.y = obstacles.collection.at(i).surface.at(floor(size/2.0)).y;
    m.pose.position.z = obstacles.collection.at(i).surface.at(floor(size/2.0)).z;
    ma_names.markers.push_back(m);
  }

  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = 1 + (int) 3*obstacles.collection.size();
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.01;  
  m.lifetime = ros::Duration(1.0/15.0);    //1/15th of a second
  m.color.a = 1.0;   m.color.r = 1.0;   m.color.g = 0.0;  m.color.b = 0.0;
  
  geometry_msgs::Point leftp;
  leftp.x = msg.left.closest_point.x; leftp.y = msg.left.closest_point.y;

  geometry_msgs::Point rightp;
  rightp.x = msg.right.closest_point.x; rightp.y = msg.right.closest_point.y;

  geometry_msgs::Point startp;
  startp.x = msg.midpoint.x - 0.5*msg.normal.x; 
  startp.y = msg.midpoint.y - 0.5*msg.normal.y;

  geometry_msgs::Point endp;
  endp.x = msg.midpoint.x + msg.normal.x; 
  endp.y = msg.midpoint.y + msg.normal.y; 

  m.points.push_back(leftp);
  m.points.push_back(rightp);
  m.points.push_back(startp);
  m.points.push_back(endp);

  obstacles_pub.publish(ma);
  obstacles_minima_pub.publish(ma_closest);
  obstacle_names_pub.publish(ma_names);
  gvg_line_pub.publish(m);
}

