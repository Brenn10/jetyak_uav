#include "ros/ros.h"

#include "../lib/bsc_common/include/util.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  std::cout << "callback called" << std::endl;
  if(!msg->markers.empty())
  {
    std::cout << "callback called" << std::endl;

    tf2::Transform transform_from_camera;
    tf2::fromMsg(msg->markers[0].pose.pose,transform_from_camera);
    std::cout << "fromMsg called" << std::endl;

    geometry_msgs::Pose pose_from_tag;
    const tf2::Transform transform_from_tag = transform_from_camera.inverse();
    std::cout << "inverse called" << std::endl;

    tf2::toMsg(transform_from_tag,pose_from_tag);
    std::cout << "toMsg called" << std::endl;

    const geometry_msgs::Quaternion* orientation = const_cast<const geometry_msgs::Quaternion*>(&pose_from_tag.orientation);
    std::cout << "cast called" << std::endl;
    ROS_WARN("x: %.2f, y: %.2f, z: %.2f",pose_from_tag.position.x,pose_from_tag.position.y,pose_from_tag.position.z);

    geometry_msgs::Vector3* state = new geometry_msgs::Vector3();
    bsc_common::util::rpy_from_quat(orientation,state);
    delete orientation;
    std::cout << "rpy_from_quat called" << std::endl;

    ROS_WARN("x: %.2f, y: %.2f, z: %.2f, yaw: %.2f",pose_from_tag.position.x,pose_from_tag.position.y,pose_from_tag.position.z,state->z);
    delete state;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"print_coords");
  ros::NodeHandle nh;

  ros::Subscriber ar_tag_sub = nh.subscribe("/ar_pose_marker",1,&arTagCallback);
  ros::spin();
  return 0;
}
