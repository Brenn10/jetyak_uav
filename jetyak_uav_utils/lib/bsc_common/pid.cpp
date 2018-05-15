#include "include/pid.h"
namespace bsc_common {
PID::PID() : PID(0.0,0.0,0.0){};

PID::PID(double kp, double ki, double kd)
{
  kp_=kp;
  kd_=kd;
  ki_=ki;
  last_error_=0;
  last_time_=0;
  integral_=0;
}

double PID::get_signal()
{
  return signal_;
}

void PID::update(double error)
{
  // Get time now
  double time_now_ = ros::Time::now().toSec();

  // Proportional
  signal_=error*kp_;

  if(last_time_!=0) //if not first time
  {
    double i,d;

    // get change in time
    double dt = last_time_-time_now_;

    // integral
    integral_+=error*dt;
    i = integral_*ki_;

    //differential
    d = kd_*(last_error_-error)/dt;

    signal_ += i+d;
  }
  last_error_=error;
  last_time_=time_now_;
}

void PID::updateParams(double kp, double ki, double kd) {
  kp_=kp;
  kd_=kd;
  ki_=ki;
}

void PID::reset()
{
  last_error_=0;
  last_time_=0;
  integral_=0;
}
} // namespace bsc_common
