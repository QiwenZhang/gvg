#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_node/Obstacle.h>
#include <laser_node/Obstacles.h>
#include <laser_node/Corners.h>
#include <boost/lexical_cast.hpp>
#include <cmath>

std::string frame_id;
ros::Publisher corners_pub;
ros::Publisher obstacles_pub;
ros::Publisher obstacles_closest_points_pub;
ros::Publisher obstacle_names_pub;
ros::Publisher bisectors_pub;
ros::Publisher centroid_pub;
void handle_corners(const laser_node::Corners::ConstPtr& msg);
void handle_obstacles(const laser_node::Obstacles::ConstPtr& msg);

int main( int argc, char** argv ) {
  ros::init(argc, argv, "laser_utils_viz");
  ros::NodeHandle nh;
  frame_id = argv[1];
  obstacle_names_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacle_names", 1);
  obstacles_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
  bisectors_pub = nh.advertise<visualization_msgs::MarkerArray>("bisectors", 1);
  obstacles_closest_points_pub = nh.advertise<visualization_msgs::Marker>("obstacles_closest_points", 1);
  centroid_pub = nh.advertise<visualization_msgs::Marker>("centroid", 1);
  ros::Subscriber obstacles_sub = nh.subscribe("/indoor/laser_utils/closest_obstacles", 1, &handle_obstacles);

  ros::spin();
  return 0;
}

void handle_obstacles(const laser_node::Obstacles::ConstPtr& msg) {
  std::string ns = "closest_obstacles";
  visualization_msgs::MarkerArray ma;
  
  /*
   * Populating obstacles
   */
  for (int i = 0; i < (int) msg->collection.size(); i++) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = msg->stamp;
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
    for (int j = 0; j < (int) msg->collection.at(i).surface.size(); j++) {
      geometry_msgs::Point pt;
      pt.x = msg->collection.at(i).surface.at(j).x;
      pt.y = msg->collection.at(i).surface.at(j).y;
      pts.push_back(pt);
    }
    
    m.points.assign(pts.begin(), pts.end());
    m.lifetime = ros::Duration(1.0/15.0);   //1/15th of a second
    ma.markers.push_back(m);
 }
  
  /*
   * Display the bisectors for all pairs of obstacles
   */
  visualization_msgs::MarkerArray bisectors;
  // sort the obstacles by their min_bearing
  std::vector<double> bearings;   
  for (int i = 0; i < (int) msg->collection.size(); i++) {
    bearings.push_back(msg->collection.at(i).min_bearing);
  }
  std::sort(bearings.begin(), bearings.end());
  std::vector<int> positions;
  for (int i = 0; i < (int) bearings.size(); i++) {
    for (int j = 0; j < (int) msg->collection.size(); j++) {
      if (msg->collection.at(j).min_bearing == bearings.at(i)) {
        positions.push_back(j);
        break;
      }
    }
  }
  for (int i = 1; i <= (int) msg->collection.size(); i++) {
    int first = positions.at(i % msg->collection.size());
    int second = positions.at(i-1);
    double ratio = msg->collection.at(first).min_distance/msg->collection.at(second).min_distance;
    double dx = msg->collection.at(first).closest_point.x - msg->collection.at(second).closest_point.x;
    double dy = msg->collection.at(first).closest_point.y - msg->collection.at(second).closest_point.y;

    geometry_msgs::Point origin; 
    origin.x = 0;
    origin.y = 0;
    
    geometry_msgs::Point p; 
    p.x = msg->collection.at(second).closest_point.x + (1.0/(ratio + 1.0))*dx;
    p.y = msg->collection.at(second).closest_point.y + (1.0/(ratio + 1.0))*dy;

    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = msg->stamp;
    m.ns = ns;
    m.id = i % msg->collection.size();
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.03;
    m.lifetime = ros::Duration(1.0/15.0);
    
    m.color.a = 1.0;   m.color.r = 1.0;   m.color.g = 0.0;  m.color.b = 1.0;

    m.points.push_back(origin);
    m.points.push_back(p);
    bisectors.markers.push_back(m);
  }

  /*
   * Calculate the centroid of a meetpoint
   */
  visualization_msgs::Marker m_centroid;
  m_centroid.header.frame_id = frame_id;
  m_centroid.header.stamp = msg->stamp;
  m_centroid.ns = ns;
  m_centroid.id = 0;
  m_centroid.type = visualization_msgs::Marker::POINTS;
  m_centroid.action = visualization_msgs::Marker::ADD;
  m_centroid.scale.x = 0.06; m_centroid.scale.y = 0.06;   
  m_centroid.lifetime = ros::Duration(1.0/15.0);

  m_centroid.color.a = 1.0;   m_centroid.color.r = 1.0;   m_centroid.color.g = 1.0;  m_centroid.color.b = 1.0;

  geometry_msgs::Point centroid;
  centroid.x = 0;
  centroid.y = 0;

  // Centroid calculated from CURRENT closest points (not accurate enough)
  for (int i = 0; i < (int) msg->collection.size(); i++) {
    centroid.x = centroid.x + msg->collection.at(i).closest_point.x;
    centroid.y = centroid.y + msg->collection.at(i).closest_point.y;
  }
  centroid.x = centroid.x/msg->collection.size();
  centroid.y = centroid.y/msg->collection.size();

  m_centroid.points.push_back(centroid);

  /*
   * Populating names of obstacles
   */
  visualization_msgs::MarkerArray ma_names;
  for (int i = 0; i < (int) msg->collection.size(); i++) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = ma.markers.at(i).header.stamp;
    m.ns = "obstacle_names";
    m.id = i;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.z = 0.2;

    m.text = boost::lexical_cast<std::string>(msg->collection.at(i).min_distance);

    m.lifetime = ma.markers.at(i).lifetime;
    m.color = ma.markers.at(i).color;
    int size = (int) msg->collection.at(i).surface.size();
    m.pose.position.x = msg->collection.at(i).surface.at(floor(size/2.0)).x;
    m.pose.position.y = msg->collection.at(i).surface.at(floor(size/2.0)).y;
    m.pose.position.z = msg->collection.at(i).surface.at(floor(size/2.0)).z;
    ma_names.markers.push_back(m);
  }


  /*
   * Display closest point on each surface with respective distance
   */
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = msg->stamp;
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.06; m.scale.y = 0.06;   
  m.lifetime = ros::Duration(1.0/15.0);

  m.color.a = 1.0;   m.color.r = 0.0;   m.color.g = 1.0;  m.color.b = 1.0;

  for (int i = 0; i < (int) msg->collection.size(); i++) {
    geometry_msgs::Point p; 
    p.x = msg->collection.at(i).closest_point.x;
    p.y = msg->collection.at(i).closest_point.y;
    p.z = msg->collection.at(i).closest_point.z;
    m.points.push_back(p); 
  }  

  obstacles_closest_points_pub.publish(m);
  obstacles_pub.publish(ma);
  obstacle_names_pub.publish(ma_names);
  bisectors_pub.publish(bisectors);
  centroid_pub.publish(m_centroid);
}

