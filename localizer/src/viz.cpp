#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <localizer/GVGmap.h>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>
#include <tf/transform_listener.h>

using namespace Eigen;
using namespace cv;
using namespace std;
#define _xdim 600.0
#define _ydim 600.0
#define _xdimI 650
#define _ydimI 650

#define margin 25
void handle_Odom(const nav_msgs::Odometry& odom);
void handle_Map(const localizer::GVGmap& map);

Mat image = Mat(_xdimI, _ydimI, CV_8UC3, cvScalar(255) );
vector<Point2f> Trajectory;

VectorXd X; // The State Vector
MatrixXd P; // The Covariance Matrix
int nL;
double minX,minY,maxX,maxY;
double Dim,_scaleX,_scaleY;
Vector3d R; // The State Vector
Matrix2d cov; // The Covariance Matrix
int cnt;
char timeS [80];



int main( int argc, char** argv ) {

  std::time_t rawtime;
  std::tm* timeinfo;
  
  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);
  std::strftime(timeS,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
  
  ros::init(argc, argv, "localizer_viz");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("/indoor/gvg/odom_combined", 1, &handle_Odom);
  ros::Subscriber map_sub  = nh.subscribe("/indoor/gvg/theMap", 1, &handle_Map);
  maxX=-1000;
  minX=1000;
  maxY=-1000;
  minY=1000;
  Dim=600;
  cnt=0;
  X=VectorXd(3);
  X<<0,0,0;
  P=MatrixXd::Zero(3,3);
  nL=0;
  cvNamedWindow("GVG Map");
  
  ros::spin();
  cvDestroyWindow("GVG Map");
  return 0;
}

Point2f scaleP(double x, double y) {
  double tmpX=margin+(_scaleX)*(x-minX);
  double tmpY=margin+_ydim-(_scaleY)*(y-minY);
  Point2f P(tmpX,tmpY);
  return(P);
}
Point2f scaleP(Point2f P) {
  return(scaleP((double)P.x,(double)P.y));
}

void drawRobot(double x, double y, double theta) {
  int thickness = 1;
  int lineType = 8;
  Point2f center=Point2f(x,y);
  Trajectory.push_back (center);
  for (std::vector<Point2f>::iterator it = Trajectory.begin() ; it != Trajectory.end(); ++it) {
    circle( image,
	    scaleP(*it),
	    1,
	    Scalar( 0, 0, 255 ),
	    thickness,
	    lineType );
  }
}
double pow2(double x) { return((x)*(x)); }

void ellipseC(double x, double y, Matrix2d cov, Scalar color=Scalar( 0, 0,250 )) {
  Point2f center=scaleP(x,y);

  EigenSolver<MatrixXd> es(cov);

  VectorXcd V1c=es.eigenvectors().col(0);
  Vector2d V1= Vector2d(std::real(V1c[0]),std::real(V1c[1])) ;
  VectorXcd V2c=es.eigenvectors().col(1);
  Vector2d V2= Vector2d(std::real(V2c[0]),std::real(V2c[1])) ;

  double l1=std::real(es.eigenvalues()[0]);
  double l2=std::real(es.eigenvalues()[1]);

  if((l1>0.0)&&(l2>0.0)) {
    double k=1.0;
    double halfaxis[2];
    double angle = -atan2(V1[1],V1[0]);
    if(l1<l2) {
      double tmp=l1; l1=l2; l2=tmp;
      angle  = -atan2(V2[1],V2[0]);
    }
    halfaxis[0]=(_scaleX)*sqrt(k*(l1))*3;
    halfaxis[1]=(_scaleY)*sqrt(k*(l2))*3;

    if((halfaxis[0]>2)&&(halfaxis[1]>2)) {
      int thickness = 1;
      int lineType = 8;


      ellipse( image,
	       center,
	       Size(halfaxis[0], halfaxis[1]),
	       angle*180.0/M_PI,
	       0,
	       360,
	       color,
	       thickness,
	       lineType );
      if (minX>x-sqrt(k*(l1))*3) minX=x-sqrt(k*(l1))*3;
      if (maxX<x+sqrt(k*(l1))*3) maxX=x+sqrt(k*(l1))*3;
      if (minY>y-sqrt(k*(l1))*3) minY=y-sqrt(k*(l1))*3;
      if (maxY<y+sqrt(k*(l1))*3) maxY=y+sqrt(k*(l1))*3;

    }
  }
  else
    cerr<<"l1 or l2 are less than 0 l1="<<l1<<" l2="<<l2<<endl;

}
void drawAll() {
  image =cvScalar(255,255,255);
  drawRobot(R(0),R(1),R(2));
  ellipseC(R(0),R(1),cov);
  for(unsigned int i=3; i<X.size(); i++) {    
    double x=X(i); i++;
    double y=X(i); i++;
    circle( image, scaleP(x,y), 4, Scalar( 30,30,30 ), 1, 8);
    ellipseC(x,y,P.block(i-2,i-2,2,2),Scalar( 255,0,0 ));
  }
  
  cv::imshow("GVG Map",image);
  cv::waitKey(1);
}

void handle_Odom(const nav_msgs::Odometry &odom) {

  R(0)=odom.pose.pose.position.x;
  R(1)=odom.pose.pose.position.y;
  tf::Pose pose;
  tf::poseMsgToTF(odom.pose.pose, pose);
  R(2)=tf::getYaw(pose.getRotation());
  if (minX>R(0)) minX=R(0);
  if (maxX<R(0)) maxX=R(0);
  if (minY>R(1)) minY=R(1);
  if (maxY<R(1)) maxY=R(1);
  double dx=maxX-minX;
  double dy=maxY-minY;
  if (dx>dy) Dim=dx;
  else Dim=dy;
  if(Dim==0) Dim=_xdim;
  _scaleX=_xdim/Dim;
  _scaleY=_ydim/Dim;

  cov<<odom.pose.covariance[0], odom.pose.covariance[1], 
    odom.pose.covariance[6], odom.pose.covariance[7];
  drawAll();
}

void handle_Map(const localizer::GVGmap& map) {

  if((unsigned)X.size()<map.state.size()) {
    X.conservativeResize(map.state.size());
  }
  nL=(map.state.size()-3)/3;
  int i=0;
  for(std::vector<double>::const_iterator it = map.state.begin();it != map.state.end(); ++it) {
    X(i)=*it;
    i++;
  }
  std::vector<double>::const_iterator it = map.cov.begin();
  unsigned int covSize=3*nL+3;
  if(P.cols()!=covSize) {
    P.conservativeResize(covSize,covSize);
  }
  for(unsigned int i=0; i<covSize; i++) {
    for(unsigned int j=0; j<covSize; j++) {
      P(i,j)=*it;
      it++;
    }
  }


  drawAll();

  char buffer [50];
  sprintf(buffer,"GVGMap%s-%04d.png",timeS,cnt);
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);

  try {
    imwrite(buffer, image, compression_params);
  }
  catch (runtime_error& ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    return;
  }
  cnt++;
}
