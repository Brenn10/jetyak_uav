#include "jetyak_uav_utils/pid.h"

pid::pid(double kp, double kd, double ki)
{
  kp_=kp;
  kd_=kd;
  ki_=ki;
  last_error_=0;
  last_time_=0;
  integral_=0;
}

double pid::get_signal()
{
  return signal_;
}

void pid::update(double error)
{
  // Proportional
  signal_=error*kp_;

  if(last_time_!=0) //if not first time
  {
    double i,d;

    // get change in time
    double dt = last_time_-ros::Time::now().toSec();

    // integral
    integral_+=error*dt;
    i = integral_*ki_;

    //differential
    d = kd_*(last_error_-error)/dt;

    signal_ += i+d;
  }
  last_error_=error;
  last_time_=ros::Time::now().toSec();
}

void pid::reset()
{
  last_error_=0;
  last_time_=0;
  integral_=0;
}
