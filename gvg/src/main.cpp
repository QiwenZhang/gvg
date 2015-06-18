#include <ros/ros.h>
#include <gvg/Access.h>
#include <gvg/FollowEdgeAction.h>
#include <gvg/SelectEdge.h>
#include <actionlib/client/simple_action_client.h>
#include <cstdlib>
#include <string>

ros::ServiceClient access_gvg_srv;
ros::ServiceClient select_edge_srv;
actionlib::SimpleActionClient<gvg::FollowEdgeAction> *follow_edge_cln;

int main(int argc, char **argv) {
  ros::init(argc, argv, "agent");
  ros::NodeHandle nh("~");

  access_gvg_srv   = nh.serviceClient<gvg::Access>("/indoor/gvg/access");
  select_edge_srv  = nh.serviceClient<gvg::SelectEdge>("/indoor/gvg/select_edge");
  follow_edge_cln  = new actionlib::SimpleActionClient<gvg::FollowEdgeAction>("/indoor/gvg/follow_edge", true);
  double lin_vel = 1.0;   // 1m/s
  double ang_vel = 40/180.0 * M_PI;  // 40 deg/s
  
  bool do_move = true;  
  nh.getParam("lin_vel", lin_vel);
  nh.getParam("ang_vel", ang_vel);  

  gvg::Access asrv;
  asrv.request.lin_vel = lin_vel;
  asrv.request.ang_vel = ang_vel;

  if (!access_gvg_srv.call(asrv)) {
    ROS_WARN("Access GVG failed");
    ros::shutdown();
    return 0;
  }
  
  ros::spinOnce();

  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
  
    nh.getParam("do_move", do_move);
    
    gvg::FollowEdgeGoal goal;
    goal.lin_vel = lin_vel;         // 1m/s, ang_vel is going to be regulated by the PID controller
    goal.do_move = do_move;   // false if we want to move the robot manually
    
    follow_edge_cln->sendGoal(goal);
    bool before_timeout = follow_edge_cln->waitForResult(ros::Duration(30*60));    
    // wait for max 30mins until the next meetpoint or endpoint

    if (!before_timeout) {
      ROS_INFO("Follow edge took too long to terminate!");
      ros::shutdown();
      return 0;
    } 

    actionlib::SimpleClientGoalState state = follow_edge_cln->getState();
    gvg::FollowEdgeResult::ConstPtr result = follow_edge_cln->getResult();
    
    switch (result->stoppedBecause) {

    case gvg::FollowEdgeResult::FOUND_MEETPOINT: {
      assert(result->node.degree >= 3);
      assert(result->node.possible_bearings.size() >= 3);
      //assert(result->node.surrounding_obstacles.collection.size() >= 3);
      assert(result->node.edge_angle_diffs.size() == result->node.possible_bearings.size());

      gvg::SelectEdge sesrv;
      sesrv.request.lin_vel = lin_vel;     // 1 m/s
      sesrv.request.ang_vel = ang_vel;    // 40 deg/s
      sesrv.request.node_id = result->node.node_id;

      if (!select_edge_srv.call(sesrv)) {
        ROS_WARN("Could not select an edge to continue");
        ros::shutdown();
        return 0;
      }

    ros::spinOnce(); }
    break;

    case gvg::FollowEdgeResult::FOUND_ENDPOINT: {
      assert(result->node.degree == 1);
      assert(result->node.possible_bearings.size() == 1);
      assert(result->node.edge_angle_diffs.size() == 1);
	 
      gvg::SelectEdge sesrv2;
      sesrv2.request.lin_vel = lin_vel;     // 1 m/s
      sesrv2.request.ang_vel = ang_vel;    // 40 deg/s
      sesrv2.request.node_id = result->node.node_id;

      if (!select_edge_srv.call(sesrv2)) {
        ROS_WARN("Could not select an edge to continue");
        ros::shutdown();
        return 0;
      }
   }
   break;

   case gvg::FollowEdgeResult::FOUND_NO_OBSTACLES:
     ROS_INFO("Found no obstacles");
     ros::shutdown();
     return 0;
	
   case gvg::FollowEdgeResult::PREEMPTED:
     ROS_INFO("Follow edge cancelled or replaced");
     break;

   default:
     ROS_ERROR("Unknown termination code for follow edge");
     ros::shutdown();
     return 0;
   }
    
   rate.sleep();
  }
  return 0;
}
