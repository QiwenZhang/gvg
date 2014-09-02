#include <cmath>
#include <cassert>
#include "BasicGeometry.h"

/* Returns a linear fit to the given points. The least square errors are vertical offsets, not
   perpendicular offsets as in the total least squares formulation. The cases where the given points
   form vertical or horizontal lines are handled. The angle of the line is in [0, 180) deg 
   and also the intercepting point on one of the two axes is also returned. */
void   fitLine(std::vector<geometry_msgs::Point32>& points, double& angle, double& intercept) {
  double sx = 0.0, sy = 0.0, stt = 0.0, sts = 0.0;
  int n = points.size();

  assert(n >= 2);
  assert(std::abs(-0.22) == 0.22);

  for (int i = 0; i < n; i++) {
    sx += points[i].x; 
    sy += points[i].y;
  }

  for (int i = 0; i < n; i++) {
    double t = points[i].x - sx/n;
    stt += t * t;
    sts += t * points[i].y;
  }

  angle = atan2(sts, stt);
  if (std::abs(stt) < 0.01) {
    angle = M_PI/2.0;
    intercept = sx/n;
  } else {
    double slope = sts/stt;
    intercept = (sy - sx*slope)/n;
  }

  bool valid = (0 <= angle && angle <= M_PI/2.0) || (0 >= angle && angle >= -M_PI/2.0);
  assert(valid);
  angle = angle * 180 / M_PI;
  
  if (angle < 0) {
    angle += 180;
  }
}

/* Returns the angle of the vector in rads */
double getAngleOfVector(geometry_msgs::Point32& from, geometry_msgs::Point32& to) {
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  return atan2(dy, dx);
}

double getAngleOfVector(geometry_msgs::Vector3& v) {
  return atan2(v.y, v.x);
}

/* Returns the euclidean norm of this vector */
double norm(geometry_msgs::Point32 from, geometry_msgs::Point32 to) {
  double x = to.x - from.x;
  double y = to.y - from.y;
  return sqrt(x*x + y*y);
}

double norm(geometry_msgs::Vector3& v) {
  return sqrt(v.x*v.x + v.y*v.y);
}

double norm(geometry_msgs::Point32& p) {
  return sqrt(p.x*p.x + p.y*p.y);
}

/* Computes the dot product of these two vectors */
double innerProduct(geometry_msgs::Point32& from1, geometry_msgs::Point32& to1, 
		    geometry_msgs::Point32& from2, geometry_msgs::Point32& to2) {

  geometry_msgs::Vector3 v1;
  v1.x = to1.x - from1.x;
  v1.y = to1.y - from1.y;

  geometry_msgs::Vector3 v2;
  v2.x = to2.x - from2.x;
  v2.y = to2.y - from2.y;

  return innerProduct(v1, v2);
}

double innerProduct(geometry_msgs::Vector3& v1, geometry_msgs::Vector3& v2) {
  return v1.x*v2.x + v1.y*v2.y;
}

/* Returns the midpoint between the two given points */
void   getMidpoint(geometry_msgs::Point32& left, geometry_msgs::Point32& right, 
		   geometry_msgs::Point32& mid) {

  mid.x = (left.x + right.x)/2.0;
  mid.y = (left.y + right.y)/2.0;
}

/* Returns true iff the two lines have the same angle up to some threshold */
bool   linearFitsHaveSameAngle(std::vector<geometry_msgs::Point32>& line1, 
			       std::vector<geometry_msgs::Point32>& line2, 
			       double same_angle_threshold) {

  assert((int) line1.size() >= 2);
  assert((int) line2.size() >= 2);

  double angleBefore, angleAfter, intercept1, intercept2; 
  fitLine(line1, angleBefore, intercept1);
  fitLine(line2, angleAfter, intercept2);
  
  double deltaAngle = std::abs(angleBefore - angleAfter);
  bool sameLineAngle = (std::abs(deltaAngle) <= same_angle_threshold || std::abs(deltaAngle - 180) <= same_angle_threshold);
  return sameLineAngle;
}

/*
 * Computes the distance of the point p to the line x(t) = a + tn 
 */
double  distanceOfPointToLine(geometry_msgs::Point32& a, geometry_msgs::Point32& n, geometry_msgs::Point32& p) {
  geometry_msgs::Vector3 ap;
  ap.x = a.x - p.x;
  ap.y = a.y - p.y;

  geometry_msgs::Vector3 nv;
  nv.x = n.x;
  nv.y = n.y;

  double ip = innerProduct(ap, nv);
  
  geometry_msgs::Point32 proj;
  proj.x = ap.x - ip * n.x;
  proj.y = ap.y - ip * n.y;

  return norm(proj);
}
