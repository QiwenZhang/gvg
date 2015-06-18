#include <iostream>
#include <limits>
#include "localizerGVG.h"
#include <tf/transform_listener.h>
#include <fstream>

localizerGVG::localizerGVG(std::string& robot_name):
  X(3),P(3,3) {
   ROS_INFO("localizerGVG::localizerGVG ");

  std::string output_frame;
  std::string theMap_frame;

  odomSub = nh.subscribe("/indoor/odom",1,&localizerGVG::handleOdom, this);
  startService = nh.advertiseService("StartFilter"    , &localizerGVG::processStart,this);
  updateService = nh.advertiseService("UpdateFilter"  , &localizerGVG::processMeetpoint,this);
  maxUncService = nh.advertiseService("MaxUncertainty", &localizerGVG::processMaxUnc,this);
  minUncService = nh.advertiseService("MinUncertainty", &localizerGVG::processMinUnc,this);
  loadMapService = nh.advertiseService("LoadLocalizerMap", &localizerGVG::loadLocalizerMap, this);
  initLoadMapTransService = nh.advertiseService("InitLoadMapTransform", &localizerGVG::initLoadMapTransform, this);

  nh.param("output_frame", output_frame, std::string("odom_combined"));
  posePub = nh.advertise<nav_msgs::Odometry>(output_frame, 10);

  nh.param("theMap_frame", theMap_frame, std::string("theMap"));

  mapPub = nh.advertise<localizer::GVGmap>(theMap_frame, 10);
  
  filterOn=true;
  initOdom=false;
  mapLoadLocalization = true;
  bearing_angle = 0;
  nL=0; // number of landmarks
  // EKF constants
  X=VectorXd(3); // The state vector initialized to the pose of the robot.
  X<< 0,0,0;

  P=MatrixXd::Zero(3,3); // The covariance matrix

  // The model noise covariance (linear and angular velocity) defined in localizer launch files
  nh.getParam("/indoor/gvg/localizerGVGNode/Wvv", this->_Wvv);
  nh.getParam("/indoor/gvg/localizerGVGNode/Www", this->_Www);
  nh.getParam("/indoor/gvg/localizerGVGNode/Wvw", this->_Wvw);
  
  Qr<< _Wvv, _Wvw,
       _Wvw, _Www;


  CF << 1,0,0,
        0,1,0,
        0,0,1;

  J << 0,-1,
       1, 0;

  R<< 0.02527455743763,   0.000014869773007,0,
    0.000014869773007,   0.02476827355463,0,
    0,0,0.025;

}

void localizerGVG::handleOdom(const nav_msgs::Odometry::ConstPtr& odom) {
  double x=odom->pose.pose.position.x;
  double y=odom->pose.pose.position.y;
  tf::Pose pose;
  tf::poseMsgToTF(odom->pose.pose, pose);
  double yaw=tf::getYaw(pose.getRotation());
  if(!initOdom) {
    oldX=x;
    oldY=y;
    oldYaw=yaw;
    initOdom=true;
    oldStamp=odom->header.stamp;
    Vector3d odomVector(x, y, yaw);
    X.segment(0, 3) = odomVector;
    return;
  }
  if(filterOn) {
    if((oldX==x)&&(oldY==y)&&(oldYaw==yaw)){
      oldStamp=odom->header.stamp;
      return;
    }
    // Calculate the measurement
    double dx=x-oldX;
    double dy=y-oldY;
    double dYaw=angleDiff(yaw,oldYaw);
    double dt=odom->header.stamp.toSec()-oldStamp.toSec();
    double Vm=sqrt(dx*dx+dy*dy)/dt;
    double Wm=dYaw/dt;
    X(0)=X(0)+Vm*dt*cos(X(2));
    X(1)=X(1)+Vm*dt*sin(X(2));
    X(2)=thetapp(X(2)+Wm*dt);
    propagate(Vm,Wm,dt);
    // Publish the odometry with covariance topic
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(X(2));
    nav_msgs::Odometry outputPose;
    outputPose.header.frame_id = "odom";
    outputPose.header.stamp =odom->header.stamp;
    outputPose.pose.pose.position.x = X(0);
    outputPose.pose.pose.position.y = X(1);
    outputPose.pose.pose.position.z = 0;
    outputPose.pose.pose.orientation=odom_quat;
    for (unsigned int i=0; i<2; i++)
      for (unsigned int j=0; j<2; j++)
    	outputPose.pose.covariance[6*i+j] = P(i,j);
    posePub.publish(outputPose);
  }
  oldX=x;
  oldY=y;
  oldYaw=yaw;
  oldStamp=odom->header.stamp;
}

void localizerGVG::propagate(double Vm, double Wm, double dt) {  
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
}



bool localizerGVG::processStart(localizer::StartFilter::Request  &req, localizer::StartFilter::Response &res) {
  if(req.start) {
    ROS_INFO("localizerGVG::StartFilter received");
  }
  else  {
    ROS_INFO("localizerGVG::StartFilter failed");
  }
  filterOn=req.start;
  return true;
}


void localizerGVG::propagateRL() {
  for(int i=0; i<nL; i++) {
    P.block(0,3+i*3,3,3)=CF*P.block(0,3+i*3,3,3);
    P.block(3+i*3,0,3,3)=P.block(3+i*3,0,3,3)*CF.transpose();
  }
  CF<<1,0,0,
      0,1,0,
      0,0,1;
}

bool localizerGVG::processMeetpoint(localizer::UpdateFilter::Request  &req,
				    localizer::UpdateFilter::Response &res) {
              
  ROS_INFO("processMeetpoint received meetpoint %d [%lf,%lf,%lf]", req.id, req.x, req.y,req.yaw);
  ROS_INFO("processMeetpoint robot pose [%lf,%lf,%lf]",X(0),X(1),X(2));
  if(nL==0) {
    //Reset the odometry covariance.
    P=MatrixXd::Zero(3,3);
  }
  // Propagate the cross-corelation between robot and the 
  // existing landmarks up to now
  propagateRL();

  // For the new landmark
  
  Vector3d xl; //the coordinates of the landmark (meetpoint) in World Frame
  Vector3d z; // Vector representation of the measurment
  z<<req.x, req.y, req.yaw;
  xl.segment(0,2)=X.segment(0,2)+C(X(2))*z.segment(0,2);
  xl(2)=X(2)+z(2);
  // Create the measurment matrices
  MatrixXd Hr(3,3);
  Vector2d dx;
  dx=xl.segment(0,2)-X.segment(0,2);
  double _c=cos(X(2));
  double _s=sin(X(2));
  Hr<<-_c, -_s, + _c*dx(1)-_s*dx(0),
    _s, -_c, - _s*dx(1)-_c*dx(0),
    0,   0, -1;
  Matrix3d Hl; // Hl is a 2 by 2 Matrix
  Hl=MatrixXd::Zero(3,3);
  Hl.block(0,0,2,2)=CT(X(2));
  Hl(2,2)=1;
  if(newLandmark(req.id)) {
    
    // If pre-fix on loaded map, need to mark that this new node is in current frame
    if (!mapLoadLocalization) preFixIdList.push_back(req.id);
          
    // New Landmark
    idList.push_back(req.id);
    P.conservativeResize(P.rows()+3,P.cols()+3);
    X.conservativeResize(X.size()+3);
    X.segment(3+nL*3,3)=xl;
    // Create new Pllcv matrices describing the cross corelation 
    // between the old landmarks and the new Landmark. Page 79 eq. 5.73 
    for (int count = 0; count < nL; count++) {
      Matrix3d newPllcv;
      newPllcv = -P.block(3 + count*3, 0, 3, 3)*Hr.transpose()*Hl;
      P.block(3 + count*3, 3 + nL*3, 3, 3) = newPllcv;
      P.block(3 + nL*3, 3 + count*3, 3, 3) = newPllcv.transpose();    
    }
    // Create new Prl matrix describing the cross corelation 
    // between the Robot and the new Landmark. Page 79 eq. 5.73 
    MatrixXd newPrl(3,3);
    
    newPrl=-P.block(0,0,3,3)*Hr.transpose()*Hl;
    P.block(0,3+nL*3,3,3)=newPrl;
    P.block(3+nL*3,0,3,3)=newPrl.transpose();
    // Create the new Pll matrix describing the covariance of the landmark
    // page 79 eq. 5.74
    Matrix3d newPll;
    newPll=Hl.transpose()*(Hr*P.block(0,0,3,3)*Hr.transpose()+R)*Hl;
    P.block(3+nL*3,3+nL*3,3,3)=newPll;
    nL++;
  }
  else {
    
    // Find which landmark it is:
    int id=-1;
    for(unsigned int i=0; i<idList.size(); i++) {
      if(idList[i] == req.id) id=i;
    }
            
    //Sanity check:
    if(id < 0) {
      cerr<<"Not able to find id:"<<req.id<< "in the list of ids"<<endl;
      for(unsigned int i=0; i<idList.size(); i++){
	      cerr<<idList[i]<<" ";
      }
      cerr<<endl;
    }
    // Check if the revisited node was one from previous map or current map
    // If from current map, don't use this node to compute transformation between old/new map
    bool oldMapNode = true;
    for (vector<int>::iterator it = preFixIdList.begin(); it != preFixIdList.end(); ++it){
      if((*it)==id) {
        oldMapNode = false;
        break;
      }
    }
    
    if (!mapLoadLocalization && oldMapNode) {
      // Copy the covariance information to Prr and also Xr
      Vector3d offset;
      offset(2) = X(2)-bearing_angle;
      Vector2d v;
      v(0) = X(3*id+3);
      v(1) = X(3*id+4);
      Vector2d rotated_mp;
      rotated_mp = C(offset(2))*v;
      offset(0) = X(0)-rotated_mp(0);
      offset(1) = X(1)-rotated_mp(1);
      
      /*P.block(0,0,3,3) = P_init.block(3*id+3, 3*id+3, 3, 3);
      // Copying information to Prl
      P.block(0,3,3,3*nL) = P.block(3*id+3, 3, 3, 3*nL); 
      P.block(3,0,3*nL,3) = P.block(3, 3*id+3, 3*nL, 3);*/
            
      mapLoadLocalization = true;
      // Calculate translation between current frame and loaded map frame
      for (unsigned int i = 0; i < nL; i++) {
        bool corrected = false;
        for (vector<int>::iterator it = preFixIdList.begin(); it != preFixIdList.end(); ++it){
          if((*it)==i) {
            corrected = true;
            break;
          }
        }
        if (!corrected) {
          Vector2d v;
          v(0) = X(3*i+3);
          v(1) = X(3*i+4);
          Vector2d result;
          result = C(offset(2))*v;
          X(3*i+3) = result(0);
          X(3*i+4) = result(1);
          
          X(3*i+3) = X(3*i+3) + offset(0);
          X(3*i+4) = X(3*i+4) + offset(1);
          X(3*i+5) = thetapp(X(3*i+5) + offset(2));
        }
      }
      
      P = P_init;
      X = X_init;
    }
    else {
      // perform update
      ROS_INFO("localizerGVG::processMeetpoint id %d %d",req.id, id);

      Vector3d z_est;
      z_est.segment(0,2)=CT(X(2))*(X.segment(3+3*id,2)-X.segment(0,2));
      z_est(2)=X(3+3*id+2)-X(2);

      Vector3d r;
      r.segment(0,2)=z.segment(0,2)-z_est.segment(0,2);
      r(2)=angleDiff(z(2),z_est(2));
      MatrixXd H(3,3+3*nL);
      H=MatrixXd::Zero(3,3+3*nL);

      H.block(0,0,3,3)=Hr;
      H.block(0,3+3*id,3,3)=Hl;
      MatrixXd S(3,3);
      S=H*P*H.transpose()+R;
      MatrixXd K(3,3+3*nL);
      K=P*H.transpose()*S.inverse();
      X=X+K*r;
      X(2)=thetapp(X(2));
      P=(MatrixXd::Identity(3+nL*3,3+nL*3)-K*H)*P*(MatrixXd::Identity(3+nL*3,3+nL*3)-K*H).transpose()+K*R*K.transpose();
    }
  }
  // publish the odometry with covariance topic    
  
  // Publish the Map
  localizer::GVGmap theMap;
  
  for(unsigned int i=0; i<X.size(); i++) {
    theMap.state.push_back(X[i]);
    res.X.push_back(X[i]);
  }

  for(unsigned int i=0; i<P.rows(); i++) {
    for(unsigned int j=0; j<P.cols(); j++) {
      theMap.cov.push_back(P(i,j));
      res.P.push_back(P(i,j));
    }
  }
  mapPub.publish(theMap);
  return(true);
}

bool localizerGVG::loadLocalizerMap(localizer::LoadLocalizerMap::Request &req, localizer::LoadLocalizerMap::Response &res) {
  
  mapLoadLocalization = false;
  
  nL = req.nL;
  X = VectorXd(3*nL+3);
  P = MatrixXd::Zero(3*nL+3, 3*nL+3);
  X_init = VectorXd(3*nL+3);
  P_init = MatrixXd::Zero(3*nL+3, 3*nL+3);
  
  for (int i = 0; i < req.ids.size(); i++) {
    idList.push_back(req.ids.at(i));
  }
  for(int i = 0; i < req.state.size(); i++) {
    X(i) = req.state.at(i);
    X_init(i) = req.state.at(i);
  }
  std::vector<double>::const_iterator it = req.cov.begin();
  int covSize = 3*nL + 3;
  for (int i = 0; i < covSize; i++) {
    for (int j = 0; j < covSize; j++) {
      P(i,j) = *it;
      P_init(i,j) = *it;
      it++;
    }
  }
  P.block(0,0,3,3) = MatrixXd::Zero(3,3);
  P.block(0,3,3,3*nL) = MatrixXd::Zero(3, 3*nL);
  P.block(3,0,3*nL,3) = MatrixXd::Zero(3*nL, 3);
  return true;
}

double localizerGVG::thetapp(double theta) {
  while(theta>M_PI) theta-=2.0*M_PI;
  while(theta<-M_PI) theta+=2.0*M_PI;
  return(theta);
}

double localizerGVG::theta02p(double theta) {
  while(theta>2*M_PI) theta-=2.0*M_PI;
  while(theta<0) theta+=2.0*M_PI;
  return(theta);
}

bool localizerGVG::newLandmark(int id){
 for (vector<int>::iterator it = idList.begin(); it != idList.end(); ++it){
    if((*it)==id) return(false);
  }
  return true;
}

double localizerGVG::angleDiff(double a, double b) {
  double diff2= theta02p(a)-theta02p(b);
  while (diff2>M_PI)  diff2-=2*M_PI; 
  while (diff2<-M_PI) diff2+=2*M_PI;
  return(diff2);
}

void localizerGVG::normalizeCovariance(){
  for(int i=0; i<P.rows()-1; i++) {
    for (int j=i+1; j<P.cols(); j++){      
      double tmp=(P(i,j)+P(j,i))/2.0;
      P(i,j)=tmp;
      P(j,i)=tmp;
    }
  }
}

Matrix2d localizerGVG::CT(double angle) {
  Matrix2d c;
  c<<cos(angle), sin(angle),
    -sin(angle), cos(angle);
  return(c);
}

Matrix2d localizerGVG::C(double angle) {
  Matrix2d c;
  c<<cos(angle), -sin(angle),
    sin(angle), cos(angle);
  return(c);
}

bool localizerGVG::initLoadMapTransform(localizer::InitLoadMapTransform::Request &req, localizer::InitLoadMapTransform::Response &res) {
  
  bearing_angle = req.bearing_angle;
  return true;
}

bool localizerGVG::processMaxUnc(localizer::MaxUncertainty::Request  &req,
				 localizer::MaxUncertainty::Response &res)  {
  double maxT=0;
  res.id=-1;
  for(int i=0; i<nL; i++) {
    double tmpT=P.block(3+i*3,3+i*3,3,3).trace();
    if(tmpT>maxT) {
      maxT=tmpT;
      res.id=i;
    }
  }
  return true;
  
}

bool localizerGVG::processMinUnc(localizer::MinUncertainty::Request  &req,
				 localizer::MinUncertainty::Response &res)  {
  double minT=1000000;
  res.id=-1;
  for(int i=0; i<nL; i++) {
    double tmpT=P.block(3+i*3,3+i*3,3,3).trace();
    if(tmpT<minT) {
      minT=tmpT;
      res.id=i;
    }
  }
  return true;
  
}
