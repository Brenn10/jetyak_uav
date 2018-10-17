#include "jetyak_uav_utils/behaviors.h"

bool behaviors::setModeCallback(jetyak_uav_utils::SetMode::Request  &req, jetyak_uav_utils::SetMode::Response &res) {

  currentMode_=(Mode)req.mode;
  behaviorChanged_=true;
  res.success=true;
  return true;
}

bool behaviors::getModeCallback(jetyak_uav_utils::GetMode::Request  &req, jetyak_uav_utils::GetMode::Response &res) {
  res.mode=(char)currentMode_;
  return true;
}

void behaviors::tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  tagPose_.pose=msg->pose;
  tagPose_.header=msg->header;
}

void behaviors::uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    uavGPS_.header=msg->header;
    uavGPS_.status=msg->status;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.longitude=msg->longitude;
    uavGPS_.altitude=msg->altitude;
    uavGPS_.position_covariance=msg->position_covariance;
    uavGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("UAV GNSS fix failed. Status: %i",msg->status.status);
  }
}

void behaviors::boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    boatGPS_.header=msg->header;
    boatGPS_.status=msg->status;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.longitude=msg->longitude;
    boatGPS_.altitude=msg->altitude;
    boatGPS_.position_covariance=msg->position_covariance;
    boatGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("Boat GNSS fix failed. Status: %i",msg->status.status);
  }
}

void behaviors::uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  uavAttitude_.header=msg->header;
  uavAttitude_.quaternion = msg->quaternion;
}

void behaviors::uavImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  uavImu_.header=msg->header;
  uavImu_.orientation=msg->orientation;
  uavImu_.orientation_covariance=msg->orientation_covariance;
  uavImu_.angular_velocity=msg->angular_velocity;
  uavImu_.angular_velocity_covariance=msg->angular_velocity_covariance;
  uavImu_.linear_acceleration=msg->linear_acceleration;
  uavImu_.linear_acceleration_covariance=msg->linear_acceleration_covariance;
}

void behaviors::boatIMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  boatImu_.header=msg->header;
  boatImu_.orientation=msg->orientation;
  boatImu_.orientation_covariance=msg->orientation_covariance;
  boatImu_.angular_velocity=msg->angular_velocity;
  boatImu_.angular_velocity_covariance=msg->angular_velocity_covariance;
  boatImu_.linear_acceleration=msg->linear_acceleration;
  boatImu_.linear_acceleration_covariance=msg->linear_acceleration_covariance;
}

void behaviors::landReconfigureCallback(jetyak_uav_utils::LandConstantsConfig &config, uint32_t level) {
  ROS_WARN("%s","Reconfigure received for Landing");

  land_.kp.x=config.kp_x;
  land_.kp.y=config.kp_y;
  land_.kp.z=config.kp_z;
  land_.kp.w=config.kp_w;

  land_.kd.x=config.kd_x;
  land_.kd.y=config.kd_y;
  land_.kd.z=config.kd_z;
  land_.kd.w=config.kd_w;

  land_.ki.x=config.ki_x;
  land_.ki.y=config.ki_y;
  land_.ki.z=config.ki_z;
  land_.ki.w=config.ki_w;

  land_.collapseRatio = config.collapse_ratio;

  if (xpid_ != NULL)
  {
    xpid_->updateParams(land_.kp.x,land_.ki.x,land_.kd.x);
    ypid_->updateParams(land_.kp.y,land_.ki.y,land_.kd.y);
    zpid_->updateParams(land_.kp.z,land_.ki.z,land_.kd.z);
    wpid_->updateParams(land_.kp.w,land_.ki.w,land_.kd.w);
  } else {
    xpid_ = new bsc_common::PID(land_.kp.x,land_.ki.x,land_.kd.x);
    ypid_ = new bsc_common::PID(land_.kp.y,land_.ki.y,land_.kd.y);
    zpid_ = new bsc_common::PID(land_.kp.z,land_.ki.z,land_.kd.z);
    wpid_ = new bsc_common::PID(land_.kp.w,land_.ki.w,land_.kd.w);
  }
}

void behaviors::followReconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level) {
  ROS_WARN("%s","Reconfigure received for follow");

  follow_.kp.x=config.kp_x;
  follow_.kp.y=config.kp_y;
  follow_.kp.z=config.kp_z;
  follow_.kp.w=config.kp_w;

  follow_.kd.x=config.kd_x;
  follow_.kd.y=config.kd_y;
  follow_.kd.z=config.kd_z;
  follow_.kd.w=config.kd_w;

  follow_.ki.x=config.ki_x;
  follow_.ki.y=config.ki_y;
  follow_.ki.z=config.ki_z;
  follow_.ki.w=config.ki_w;

  follow_.follow_pose.x = config.follow_x;
  follow_.follow_pose.y = config.follow_y;
  follow_.follow_pose.z = config.follow_z;
  follow_.follow_pose.w = config.follow_w;

  if (xpid_ != NULL)
  {
    xpid_->updateParams(follow_.kp.x,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w,follow_.ki.w,follow_.kd.w);
  } else {
    xpid_ = new bsc_common::PID(follow_.kp.x,follow_.ki.x,follow_.kd.x);
    ypid_ = new bsc_common::PID(follow_.kp.y,follow_.ki.y,follow_.kd.y);
    zpid_ = new bsc_common::PID(follow_.kp.z,follow_.ki.z,follow_.kd.z);
    wpid_ = new bsc_common::PID(follow_.kp.w,follow_.ki.w,follow_.kd.w);
  }
}
