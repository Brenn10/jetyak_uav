#ifndef BSC_COMMON_UTIL_
#define BSC_COMMON_UTIL_


#include <tf/transform_datatypes.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"
namespace bsc_common {

const long double C_PI = 3.14159265358979323846; // My long PI value

class util {
public:
  /* xyzw_from_pose
   * Gives the x,y,z,yaw offset given a pose.
   * xyz are extracted and yaw is calculated from Quaternion
   *
   * @param orientation quaternion
   * @param state saves the rollmpitch,yaw encoded as a Vector3
   */
  static void rpy_from_quat(const geometry_msgs::Quaternion* pose, geometry_msgs::Vector3* state);

  /* clip
   * clips a value to be between a min and maxSpeed
   * equivalent to max(min(high,x),low)
   *
   * @param x value to be clipped
   * @param low lower bound
   * @param high upper bound
   *
   * @return clipped value of x
  */
  static float clip(float x, float low, float high) {
    return std::max(std::min(high,x),low);
  };
};

} // namespace bsc_common
#endif
