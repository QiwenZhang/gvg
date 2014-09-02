#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>

class PIDController {
 public:
  PIDController(double signal, double Kp, double Kd, double Ki);
 
  void update(double error);
 
  double signal;
  double desiredSignal;
  double Kp;
  double Kd;
  double Ki;
  double curr_error;
  double prev_error;
  double error_sum;
  ros::Time prev;
};

#endif
