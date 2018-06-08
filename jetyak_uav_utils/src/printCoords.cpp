#include "ros/ros.h"

#include "../lib/bsc_common/include/util.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if(!msg->markers.empty())
  {
    tf2::Transform transform_from_camera;
    tf2::fromMsg(msg->markers[0].pose.pose,transform_from_camera);
    geometry_msgs::Vector3* state;
    geometry_msgs::Pose pose_from_tag;
    const tf2::Transform transform_from_tag = transform_from_camera.inverse();
    tf2::toMsg(transform_from_tag,pose_from_tag);

    const geometry_msgs::Quaternion* orientation = const_cast<const geometry_msgs::Quaternion*>(&pose_from_tag.orientation);
    bsc_common::util::rpy_from_quat(orientation,state);

    ROS_WARN("x: %.2f, y: %.2f, z: %.2f, yaw: %.2f",pose_from_tag.position.x,pose_from_tag.position.y,pose_from_tag.position.z,state->z);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"print_coords");
  ros::NodeHandle nh;

  ros::Subscriber ar_tag_sub = nh.subscribe("/ar_track_alvar",1,&arTagCallback);
  ros::spin();
  return 0;
}
