#include <iostream>
#include <limits>
#include "ekf_sim.h"
#include <iostream>
#include <fstream>

ekf_sim::ekf_sim(double Wvv, double Wvw, double Www, double a) {
    
  this->retrieve_path_cln = nh.serviceClient<gvg_mapper::RetrievePath>("/retrieve_path");
    
  initOdom = false;  
  total_distance = 0.0;
  alpha = a;
    
  nL = 0; // number of landmarks
  // EKF constants
  X=VectorXd(3); // The state vector initialized to the pose of the robot.
  X << 0,0,0;

  P=MatrixXd::Zero(3,3); // The covariance matrix
  // The model noise covariance (linear and angular velocity)

  Qr << Wvv, Wvw,
        Wvw, Www;

  CF << 1,0,0,
        0,1,0,
        0,0,1;

  J << 0,-1,
       1, 0;

  R << 0.02527455743763,   0.000014869773007,0,
       0.000014869773007,   0.02476827355463,0,
       0,0,0.025;
       
}


ekf_sim::ekf_sim(double Wvv, double Wvw, double Www, double a, int nL, VectorXd state, MatrixXd cov, int current_node) {
  
  this->retrieve_path_cln = nh.serviceClient<gvg_mapper::RetrievePath>("/retrieve_path");
  
  initOdom = false;
  total_distance = 0.0;
  alpha = a;
  
  // The model noise covariance (linear and angular velocity)
  Qr << Wvv, Wvw,
        Wvw, Www;

  CF << 1,0,0,
        0,1,0,
        0,0,1;

  J << 0,-1,
       1, 0;

  R << 0.02527455743763,   0.000014869773007,0,
       0.000014869773007,   0.02476827355463,0,
       0,0,0.025;
       
  this->nL = nL;
  X = VectorXd(3*nL+3);
  P = MatrixXd::Zero(3*nL+3, 3*nL+3);
  unsigned int covSize = 3*nL+3;
  
  for(int i = 0; i < covSize; i++) {
    X(i) = state(i);
  }
  X(0) = X(3*current_node + 3);
  X(1) = X(3*current_node + 3);
  for(unsigned int i = 0; i < covSize; i++) {
    for(unsigned int j = 0; j < covSize; j++) {
      P(i,j) = cov(i,j);
    }
  }
  
  start_map_trace = P.block(3, 3, 3*nL, 3*nL).trace();
}

void ekf_sim::copy(ekf_sim ekf) {
  
  initOdom = false;
  total_distance = ekf.total_distance;
  
  nL = ekf.nL;
  X = VectorXd(3*nL+3);
  P = MatrixXd::Zero(3*nL+3, 3*nL+3);
  
  for(int i = 0; i < ekf.X.size(); i++) {
    X(i) = ekf.X(i);
  }
  for(unsigned int i = 0; i < ekf.P.rows(); i++) {
    for(unsigned int j = 0; j < ekf.P.cols(); j++) {
      P(i,j) = ekf.P(i,j);
    }
  }
  
  /*for (int i = 0; i < ekf.sim_nodes.size(); i++) {
    sim_nodes.push_back(ekf.sim_nodes.at(i));
  }*/
  
  CF << 1,0,0,
        0,1,0,
        0,0,1;
        
  start_map_trace = ekf.start_map_trace;
  start_shortest_distance = ekf.start_shortest_distance;
}

void ekf_sim::propagate(geometry_msgs::PointStamped p_stamped) {
  
  double x = p_stamped.point.x;
  double y = p_stamped.point.y;
  // Orientation stored in z component
  double yaw = p_stamped.point.z;
  if(!initOdom) {
    oldX = x;
    oldY = y;
    oldYaw = yaw;
    initOdom = true;
    oldStamp = p_stamped.header.stamp;
    Vector3d odomVector(x, y, yaw);
    X.segment(0, 3) = odomVector;
    return;
  }
  
  if (((oldX == x) && (oldY == y) && (oldYaw == yaw)) || fabs(p_stamped.header.stamp.toSec() - oldStamp.toSec()) == 0) {
    oldStamp = p_stamped.header.stamp;
    return;
  }
  // Calculate the measurement
  double dx = x - oldX;
  double dy = y - oldY;
  double dYaw = angleDiff(yaw,oldYaw);
  double dt = fabs(p_stamped.header.stamp.toSec() - oldStamp.toSec());
  double Vm = sqrt(dx*dx+dy*dy)/dt;
  double Wm = dYaw/dt;
  X(0) = X(0) + Vm*dt*cos(X(2));
  X(1) = X(1) + Vm*dt*sin(X(2));
  X(2) = thetapp(X(2)+Wm*dt);
  
  Matrix3d Fr;
  Fr<<1, 0, -Vm*dt*sin(oldYaw),
      0, 1,  Vm*dt*cos(oldYaw),
      0, 0,  1;
  MatrixXd Gr(3,2);
  Gr<< -dt*cos(oldYaw), 0, 
       -dt*sin(oldYaw), 0,
        0,             -dt;

  P.block(0,0,3,3)=Fr*P.block(0,0,3,3)*Fr.transpose()+Gr*Qr*Gr.transpose();
  CF=Fr*CF;
  normalizeCovariance();
  
  oldX = x;
  oldY = y;
  oldYaw = yaw;
  oldStamp = p_stamped.header.stamp;
}

void ekf_sim::propagateRL() {
  for(int i = 0; i < nL; i++) {
    P.block(0,3+i*3,3,3) = CF*P.block(0,3+i*3,3,3);
    P.block(3+i*3,0,3,3) = P.block(3+i*3,0,3,3)*CF.transpose();
  } 
  CF << 1,0,0,
        0,1,0,
        0,0,1;
}

void ekf_sim::update(int node_id, geometry_msgs::Point32 pl, geometry_msgs::Point32 measurement) {

  // Propagate the cross-corelation between robot and the 
  // existing landmarks up to now
  propagateRL();
    
  Vector3d xl; //the coordinates of the landmark (meetpoint) in World Frame
  Vector3d z; // Vector representation of the measurment
  z << measurement.x, measurement.y, M_PI;
  xl << pl.x, pl.y, oldYaw;
  X.segment(0,2) = xl.segment(0,2) - C(X(2))*z.segment(0,2);
  xl(2) = X(2) + z(2);
  // Create the measurement matrices
  MatrixXd Hr(3,3);
  Vector2d dx;
  dx = xl.segment(0,2) - X.segment(0,2);
  double _c = cos(X(2));
  double _s = sin(X(2));
  Hr << -_c, -_s, + _c*dx(1)-_s*dx(0),
    _s, -_c, - _s*dx(1)-_c*dx(0),
    0,   0, -1;
  Matrix3d Hl; // Hl is a 2 by 2 Matrix
  Hl = MatrixXd::Zero(3,3);
  Hl.block(0,0,2,2) = CT(X(2));
  Hl(2,2) = 1;
  
  // perform update

  Vector3d z_est;
  z_est.segment(0,2) = CT(X(2))*(X.segment(3+3*node_id,2) - X.segment(0,2));
  z_est(2) = X(3+3*node_id+2) - X(2);

  Vector3d r;
  r.segment(0,2) = z.segment(0,2)-z_est.segment(0,2);
  r(2) = angleDiff(z(2),z_est(2));
  MatrixXd H(3,3+3*nL);
  H=MatrixXd::Zero(3,3+3*nL);

  H.block(0,0,3,3) = Hr;
  H.block(0,3+3*node_id,3,3) = Hl;
  MatrixXd S(3,3);
  S = H*P*H.transpose() + R;
  MatrixXd K(3,3+3*nL);
  K = P*H.transpose()*S.inverse();
  X = X+K*r;
  X(2) = thetapp(X(2));
  P = (MatrixXd::Identity(3+nL*3,3+nL*3) - K*H)*P*(MatrixXd::Identity(3+nL*3,3+nL*3) - K*H).transpose()+K*R*K.transpose();
}

double ekf_sim::computeCost(int target) {
  double g = (alpha*(total_distance))/start_shortest_distance;
  double unc = (1.0-alpha)*(P.block(3, 3, 3*nL, 3*nL).trace()/start_map_trace);
  double h = (alpha*sqrt( (X(3*target+3)-X(3*current_node+3))*(X(3*target+3)-X(3*current_node+3)) + (X(3*target+4)-X(3*current_node+4))*(X(3*target+4)-X(3*current_node+4)) ))/start_shortest_distance;
  gvg_mapper::RetrievePath pathsrv;
  pathsrv.request.source = current_node;
  pathsrv.request.target = target;
  if (!retrieve_path_cln.call(pathsrv)) {
    ROS_WARN("Could not retrieve shortest distance to target from Mapper!");
  }
  else {
    h = (alpha*pathsrv.response.distance)/start_shortest_distance;
  }
  return (g + unc + h);
}

double ekf_sim::thetapp(double theta) {
  while(theta>M_PI) theta-=2.0*M_PI;
  while(theta<-M_PI) theta+=2.0*M_PI;
  return(theta);
}

double ekf_sim::theta02p(double theta) {
  while(theta>2*M_PI) theta-=2.0*M_PI;
  while(theta<0) theta+=2.0*M_PI;
  return(theta);
}

double ekf_sim::angleDiff(double a, double b) {
  double diff2= theta02p(a)-theta02p(b);
  while (diff2>M_PI)  diff2-=2*M_PI; 
  while (diff2<-M_PI) diff2+=2*M_PI;
  return(diff2);
}

void ekf_sim::normalizeCovariance(){
  for(int i=0; i<P.rows()-1; i++) {
    for (int j=i+1; j<P.cols(); j++){      
      double tmp=(P(i,j)+P(j,i))/2.0;
      P(i,j)=tmp;
      P(j,i)=tmp;
    }
  }
}

Matrix2d ekf_sim::CT(double angle) {
  Matrix2d c;
  c<<cos(angle), sin(angle),
    -sin(angle), cos(angle);
  return(c);
}

Matrix2d ekf_sim::C(double angle) {
  Matrix2d c;
  c<<cos(angle), -sin(angle),
    sin(angle), cos(angle);
  return(c);
}
