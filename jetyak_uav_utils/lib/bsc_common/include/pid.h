#ifndef BSC_COMMON_PID_
#define BSC_COMMON_PID_
#include "ros/ros.h"
namespace bsc_common {
class PID {
private:
  double kp_,ki_,kd_,last_error_,integral_,last_time_,signal_=0;
public:
  PID();
  PID(double kp, double ki, double kd);
  void update(double error);
  void updateParams(double kp, double ki, double kd);
  void reset();
  double get_signal();
};
} // namespace bsc_common

#endif
