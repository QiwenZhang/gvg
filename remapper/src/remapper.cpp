#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

//This node simply listens to the LaserScan message coming from the Husky, and reverses the order of the ranges.
//This is to compensate for the fact that the laser is mounted upside-down.

class ReMapper {
public:
  
	ReMapper(ros::NodeHandle& nh, bool pub_reversed_laser) { 
    if (pub_reversed_laser) {
      laser_sub = nh.subscribe("/lidar/scan", 1, &ReMapper::laserCallBack, this);
      laser_pub = nh.advertise<sensor_msgs::LaserScan>("/lidar/reversed", 1);
    } 
		pose_sub  = nh.subscribe("/robot_pose_ekf/odom", 1, &ReMapper::poseCallBack, this);
		odom_pub  = nh.advertise<nav_msgs::Odometry>("/indoor/pose_to_odom", 1);
	}
	
	void poseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    nav_msgs::Odometry output;
    output.header = msg->header;
    output.pose = msg->pose;
    odom_pub.publish(output);
	}
	
	void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg) {
		unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
    unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
		sensor_msgs::LaserScan reversed;
		reversed.header = msg->header;
		reversed.angle_min = msg->angle_min;
		reversed.angle_max = msg->angle_max;
		reversed.angle_increment = msg->angle_increment;
		reversed.time_increment = msg->time_increment;
		reversed.scan_time = msg->scan_time;
		reversed.range_min = msg->range_min;
		reversed.range_max = msg->range_max;
		for(unsigned int i = 0; i < minIndex; i++) {
			reversed.ranges.push_back(0);
			reversed.intensities.push_back(0);
		}
		for(unsigned int i = 0; i <= maxIndex; i++){
			reversed.ranges.push_back(msg->ranges[maxIndex - i]);
			reversed.intensities.push_back(msg->intensities[maxIndex - i]);
		}
    laser_pub.publish(reversed);
	}
	
	void spin() {
		while(ros::ok()){
			ros::spinOnce();
		}
		ros::shutdown();
	}
	
	const static double MIN_SCAN_ANGLE_RAD = -135.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +135.0/180*M_PI;
	
protected:
	ros::Publisher laser_pub;
	ros::Subscriber laser_sub;
	ros::Publisher odom_pub;
	ros::Subscriber pose_sub;
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "remapper");
	ros::NodeHandle nh;
  bool pub_reversed_laser = false;
  nh.getParam("/remapper/reverse_laser", pub_reversed_laser);
	ReMapper remapper(nh, pub_reversed_laser);
	remapper.spin();
  return 0;
}
