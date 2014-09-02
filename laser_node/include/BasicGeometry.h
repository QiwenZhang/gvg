#ifndef BASIC_GEOMETRY_H
#define BASIC_GEOMETRY_H

#include <vector>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>


/* Returns the angle of the vector in rads */
double getAngleOfVector(geometry_msgs::Point32& from, geometry_msgs::Point32& to);
double getAngleOfVector(geometry_msgs::Vector3& v);

/* Returns the midpoint between the two given points */
void   getMidpoint(geometry_msgs::Point32& left, geometry_msgs::Point32& right, geometry_msgs::Point32& mid);

/* Returns a linear fit to the given points. The least square errors are vertical offsets, not
   perpendicular offsets as in the total least squares formulation. The cases where the given points
   form vertical or horizontal lines are handled. The angle of the line is in [0, 180) deg 
   and also the intercepting point on one of the two axes is also returned. */
void   fitLine(std::vector<geometry_msgs::Point32>& points, double& angle, double& intercept);

/* Returns the euclidean norm of this vector */
double norm(geometry_msgs::Point32 from, geometry_msgs::Point32 to);
double norm(geometry_msgs::Vector3& v);
double norm(geometry_msgs::Point32& p);

/* Computes the dot product of these two vectors */
double innerProduct(geometry_msgs::Point32& from1, geometry_msgs::Point32& to1, 
		    geometry_msgs::Point32& from2, geometry_msgs::Point32& to2); 

double innerProduct(geometry_msgs::Vector3& v1, geometry_msgs::Vector3& v2); 

/* Returns true iff the two lines have the same angle up to some threshold */
bool   linearFitsHaveSameAngle(std::vector<geometry_msgs::Point32>& line1, 
			       std::vector<geometry_msgs::Point32>& line2, 
			       double same_angle_threshold);

/*
 * Computes the distance of the point p to the line x(t) = a + tn 
 */
double  distanceOfPointToLine(geometry_msgs::Point32& a, geometry_msgs::Point32& n, 
			      geometry_msgs::Point32& p);
#endif 
