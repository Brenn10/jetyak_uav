#include "include/util.h"
namespace bsc_common {
void util::xyzw_from_pose(const geometry_msgs::Pose* pose, geometry_msgs::Quaternion* state) {
  state->x = pose->position.x;
  state->y = pose->position.y;
  state->z = pose->position.z;

  // Get message as quaternion, then get roll, pitch, yaw
  tf::Quaternion q(
    pose->orientation.x,
    pose->orientation.y,
    pose->orientation.z,
    pose->orientation.w
  );
  tf::Matrix3x3 m(q);
  double t_r, t_p, t_y;
  m.getRPY(t_r, t_p, t_y);

  state->w = t_y;

  if(state->w>C_PI) {
    state->w = state->w-2*C_PI;
  }
}
} // namespace bsc_common
