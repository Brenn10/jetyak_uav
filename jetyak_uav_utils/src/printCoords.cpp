#include "ros/ros.h"

#include "../lib/bsc_common/include/util.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
  if (!msg->markers.empty()) {
    geometry_msgs::Pose pose_from_tag;
    bsc_common::util::inverse_pose(msg->markers[0].pose.pose, pose_from_tag);
    const geometry_msgs::Quaternion *orientation =
        const_cast<const geometry_msgs::Quaternion *>(
            &pose_from_tag.orientation);
    geometry_msgs::Vector3 *state = new geometry_msgs::Vector3();
    bsc_common::util::rpy_from_quat(orientation, state);
    double yaw = state->z + bsc_common::util::C_PI / 2;
    if (yaw > bsc_common::util::C_PI) {
      yaw = yaw - 2 * bsc_common::util::C_PI;
    }
    ROS_WARN("x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", pose_from_tag.position.x,
             pose_from_tag.position.y, pose_from_tag.position.z, yaw);
    double rotated_x;
    double rotated_y;

    delete state;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "print_coords");
  ros::NodeHandle nh;

  ros::Subscriber ar_tag_sub =
      nh.subscribe("/ar_pose_marker", 1, &arTagCallback);
  ros::spin();
  return 0;
}
