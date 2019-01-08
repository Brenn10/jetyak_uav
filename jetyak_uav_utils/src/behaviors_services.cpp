#include "jetyak_uav_utils/behaviors.h"

bool behaviors::setModeCallback(jetyak_uav_utils::SetMode::Request &req,
                                jetyak_uav_utils::SetMode::Response &res) {

  currentMode_ = (Mode)req.mode;
  behaviorChanged_ = true;
  res.success = true;
  ROS_WARN("Mode changed to %s", nameFromMode[(int)currentMode_].c_str());
  return true;
}

bool behaviors::getModeCallback(jetyak_uav_utils::GetMode::Request &req,
                                jetyak_uav_utils::GetMode::Response &res) {
  res.mode = (char)currentMode_;
  return true;
}

bool behaviors::setBoatNSCallback(jetyak_uav_utils::SetBoatNS::Request &req,
                                  jetyak_uav_utils::SetBoatNS::Response &res) {
  std::string ns = req.ns;
  boatGPSSub_ = nh.subscribe(ns + "/global_posiiton/global", 1,
                             &behaviors::boatGPSCallback, this);
  boatIMUSub_ =
      nh.subscribe(ns + "/imu/data", 1, &behaviors::boatIMUCallback, this);
  return true;
}

bool behaviors::setFollowPIDCallback(
    jetyak_uav_utils::FourAxes::Request &req,
    jetyak_uav_utils::FourAxes::Response &res) {
  // If following, taking off, or returning, change active controller
  if (currentMode_ == Mode::FOLLOW or currentMode_ == Mode::TAKEOFF or
      currentMode_ == Mode::RETURN) {
    xpid_->updateParams(req.x[0], req.x[1], req.x[2]);
    ypid_->updateParams(req.y[0], req.y[1], req.y[2]);
    zpid_->updateParams(req.z[0], req.z[1], req.z[2]);
    wpid_->updateParams(req.w[0], req.w[1], req.w[2]);
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
  }

  // either way, change stored values
  follow_.kp.x = req.x[0];
  follow_.ki.x = req.x[1];
  follow_.kd.x = req.x[2];

  follow_.kp.y = req.y[0];
  follow_.ki.y = req.y[1];
  follow_.kd.y = req.y[2];

  follow_.kp.z = req.z[0];
  follow_.ki.z = req.z[1];
  follow_.kd.z = req.z[2];

  follow_.kp.w = req.w[0];
  follow_.ki.w = req.w[1];
  follow_.kd.w = req.w[2];

  res.success = true;
  return true;
}

bool behaviors::setLandPIDCallback(jetyak_uav_utils::FourAxes::Request &req,
                                   jetyak_uav_utils::FourAxes::Response &res) {
  // If landing, change active controller
  if (currentMode_ == Mode::LAND) {
    xpid_->updateParams(req.x[0], req.x[1], req.x[2]);
    ypid_->updateParams(req.y[0], req.y[1], req.y[2]);
    zpid_->updateParams(req.z[0], req.z[1], req.z[2]);
    wpid_->updateParams(req.w[0], req.w[1], req.w[2]);
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();
  }

  // either way, change stored values
  land_.kp.x = req.x[0];
  land_.ki.x = req.x[1];
  land_.kd.x = req.x[2];

  land_.kp.y = req.y[0];
  land_.ki.y = req.y[1];
  land_.kd.y = req.y[2];

  land_.kp.z = req.z[0];
  land_.ki.z = req.z[1];
  land_.kd.z = req.z[2];

  land_.kp.w = req.w[0];
  land_.ki.w = req.w[1];
  land_.kd.w = req.w[2];

  res.success = true;
  return true;
}

bool behaviors::setFollowPositionCallback(
    jetyak_uav_utils::FourAxes::Request &req,
    jetyak_uav_utils::FourAxes::Response &res) {
  follow_.follow_pose.x = req.x[0];
  follow_.follow_pose.y = req.y[0];
  follow_.follow_pose.z = req.z[0];
  follow_.follow_pose.w = req.w[0];

  res.success = true;
  return true;
}

bool behaviors::setLandPositionCallback(
    jetyak_uav_utils::FourAxes::Request &req,
    jetyak_uav_utils::FourAxes::Response &res) {
  land_.land_pose.x = req.x[0];
  land_.land_pose.y = req.y[0];
  land_.land_pose.z = req.z[0];
  land_.land_pose.w = req.w[0];

  res.success = true;
  return true;
}

bool behaviors::setTakeoffParamsCallback(
    jetyak_uav_utils::TakeoffParams::Request &req,
    jetyak_uav_utils::TakeoffParams::Response &res) {
  takeoff_.height = req.height;
  takeoff_.threshold = req.threshold;

  res.success = true;
  return true;
}

bool behaviors::setLandParamsCallback(
    jetyak_uav_utils::LandParams::Request &req,
    jetyak_uav_utils::LandParams::Response &res) {
  land_.height_goal = req.height_goal;
  land_.height_min = req.height_min;
  land_.radius = req.radius;

  res.success = true;
  return true;
}

bool behaviors::setReturnParamsCallback(
    jetyak_uav_utils::ReturnParams::Request &req,
    jetyak_uav_utils::ReturnParams::Response &res) {
  return_.gotoHeight = req.gotoHeight;
  return_.finalHeight = req.finalHeight;
  return_.downRadius = req.downRadius;
  return_.settleRadiusSquared = req.settleRadius * req.settleRadius;
  return_.tagTime = req.tagTime;
  return_.tagLossThresh = req.tagLossThresh;
  res.success = true;
  return true;
}