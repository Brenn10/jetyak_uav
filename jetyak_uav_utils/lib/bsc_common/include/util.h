#ifndef BSC_COMMON_UTIL_
#define BSC_COMMON_UTIL_


#include <tf/transform_datatypes.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"
namespace bsc_common {

const long double C_PI = 3.14159265358979323846; // My long PI value

class util {
public:
  static void xyzw_from_pose(const geometry_msgs::Pose* pose, geometry_msgs::Quaternion* state);
  static float clip(float x, float low, float high) {
    return std::max(std::min(high,x),low);
  };
};

} // namespace bsc_common
#endif
