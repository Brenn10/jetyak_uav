#ifndef JETYAK_UAV_UTILS_PID_H_
#define JETYAK_UAV_UTILS_PID_H_

#include "ros/ros.h"

class PID {
  private:
    double kp_,ki_,kd_,last_error_,integral_,last_time_,signal_;
  public:

    /** Constructor
    * initialize the variables
    *
    * @param kp proportional constant
    * @param ki integral constant
    * @param kd differential constant
    */
    pid(double kp, double ki, double kd);

    /** update
    * Update the signal variables
    *
    * @param error Error input to the PID function
    */
    void update(double error);

    /** reset
    * reset all of the variables
    */
    void reset();

    /** get_signal
    * Gives the signal variables
    *
    * @return requested signal
    */
    double get_signal();
};

#endif
