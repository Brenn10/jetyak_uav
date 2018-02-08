#ifndef JETYAK_UAV_UTILS_PID_H_
#define JETYAK_UAV_UTILS_PID_H_

#include "ros/ros.h"

class pid {
  private:
    double kp_,ki_,kd_,last_error_,integral_,last_time_,signal_;
  public:

    pid(double kp, double ki, double kd);
    void update(double error);
    void reset();
    double get_signal();
};

#endif
