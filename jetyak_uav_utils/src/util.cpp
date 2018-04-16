#include "../include/jetyak_uav_utils/util.h"
namespace bsc_common {
void utils::xyzw_from_pose(geometry_msgs::Pose &pose, geometry_msgs::Quaternion state) {
  state.x = pose.pose.position.x;
  state.y = pose.pose.position.y;
  state.z = pose.pose.position.z;

  // Get message as quaternion, then get roll, pitch, yaw
  tf::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w
  );
  tf::Matrix3x3 m(q);
  double t_r, t_p, t_y;
  m.getRPY(t_r, t_p, t_y);

  state.w = t_y;

  if(state.w>C_PI) {
    state.w = state.w-2*C_PI;
  }
}
