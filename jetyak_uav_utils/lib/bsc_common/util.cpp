#include "include/util.h"
#include <iostream>
namespace bsc_common {
void util::rpy_from_quat(const geometry_msgs::Quaternion* orientation, geometry_msgs::Vector3* state) {


  // Get message as quaternion, then get roll, pitch, yaw
  tf::Quaternion q(
    orientation->x,
    orientation->y,
    orientation->z,
    orientation->w
  );
  tf::Matrix3x3 m(q);
  double t_r, t_p, t_y;
  m.getRPY(t_r, t_p, t_y);
  std::cout<<"r: " <<t_r<<", p: "<<t_p<<", y: "<<t_y<<std::endl;
  state->x = t_r;
  state->y = t_p;
  state->z = t_y;
  std::cout <<"wrote" <<std::endl;

  if(state->x>C_PI) {
    state->x = state->x-2*C_PI;
  }
  if(state->y>C_PI) {
    state->y = state->y-2*C_PI;
  }
  if(state->z>C_PI) {
    state->z = state->z-2*C_PI;
  }
  std::cout <<"close" <<std::endl;
}

float util::clip(float x, float low, float high) {
  return std::max(std::min(high,x),low);
}

void util::rotate_vector(double x, double y, double theta,double *xp,double *yp) {
  *xp = (x*std::cos(theta)-y*std::sin(theta));
  *yp = (x*std::sin(theta)+y*std::cos(theta));
}

} // namespace bsc_common
