#include "PIDController.h"
#include <ros/ros.h>

PIDController::PIDController(double signal, double Kp, double Kd, double Ki) {
  this->signal = signal;
  this->desiredSignal = signal; 
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
  this->curr_error = 0;
  this->prev_error = 0;
  this->error_sum = 0;
  this->prev = ros::Time::now();
}

void PIDController::update(double error) {
  
  static bool firstTime = true;
  
  ros::Time now = ros::Time::now();
  ros::Duration dt = now - prev;
  prev_error = curr_error;
  curr_error = error;
  
  if (!firstTime) {
    error_sum += error * dt.toSec();
    double dError = (curr_error - prev_error)/dt.toSec();
    signal = desiredSignal + Kp * curr_error + Kd * dError + Ki*error_sum; 
  } else {
    signal = desiredSignal + Kp * curr_error; 
  }

  firstTime = false;
  prev = now;
}
