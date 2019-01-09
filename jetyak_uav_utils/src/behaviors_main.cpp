#include "jetyak_uav_utils/behaviors.h"

Behaviors::Behaviors(ros::NodeHandle &nh_param) {
  nh = nh_param;
  // initialize mode
  currentMode_ = JETYAK_UAV::HOVER;

  // subscribers
  tagPoseSub_ = nh.subscribe("/tag_pose", 1, &Behaviors::tagPoseCallback, this);
  uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position", 1,
                            &Behaviors::uavGPSCallback, this);
  boatGPSSub_ = nh.subscribe("boat_gps", 1, &Behaviors::boatGPSCallback, this);
  uavAttSub_ = nh.subscribe("/dji_sdk/attitude", 1,
                            &Behaviors::uavAttitudeCallback, this);
  boatIMUSub_ = nh.subscribe("boat_imu", 1, &Behaviors::boatIMUCallback, this);

  // Publishers
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("behavior_cmd", 1);

  // service clients
  propSrv_ = nh.serviceClient<jetyak_uav_utils::Boolean>("prop_enable");
  takeoffSrv_ = nh.serviceClient<std_srvs::Trigger>("takeoff");
  landSrv_ = nh.serviceClient<std_srvs::Trigger>("land");

  // Service servers
  setModeService_ =
      nh.advertiseService("setMode", &Behaviors::setModeCallback, this);
  getModeService_ =
      nh.advertiseService("getMode", &Behaviors::getModeCallback, this);
  setBoatNSService_ =
      nh.advertiseService("setBoatNS", &Behaviors::setBoatNSCallback, this);
  setFollowPIDService_ = nh.advertiseService(
      "setFollowPID", &Behaviors::setFollowPIDCallback, this);
  setLandPIDService_ =
      nh.advertiseService("setLandPID", &Behaviors::setLandPIDCallback, this);
  setFollowPosition_ = nh.advertiseService(
      "setFollowPosition", &Behaviors::setFollowPositionCallback, this);
  setLandPosition_ = nh.advertiseService(
      "setLandPosition", &Behaviors::setLandPositionCallback, this);
  setTakeoffParams_ = nh.advertiseService(
      "setTakeoffParams", &Behaviors::setTakeoffParamsCallback, this);
  setReturnParams_ = nh.advertiseService(
      "setReturnParams", &Behaviors::setReturnParamsCallback, this);
  setLandParams_ = nh.advertiseService("setLandParams",
                                       &Behaviors::setLandParamsCallback, this);
  ;

  /****************************************
   * ASSIGNING ROS PARAMETERS TO THE NODE
   ***************************************/
  // land pid
  ros::param::param<double>("behaviors/land_x_kp", land_.kp.x, 0);
  ros::param::param<double>("behaviors/land_y_kp", land_.kp.y, 0);
  ros::param::param<double>("behaviors/land_z_kp", land_.kp.z, 0);
  ros::param::param<double>("behaviors/land_w_kp", land_.kp.w, 0);

  ros::param::param<double>("behaviors/land_x_kd", land_.kd.x, 0);
  ros::param::param<double>("behaviors/land_y_kd", land_.kd.y, 0);
  ros::param::param<double>("behaviors/land_z_kd", land_.kd.z, 0);
  ros::param::param<double>("behaviors/land_w_kd", land_.kd.w, 0);

  ros::param::param<double>("behaviors/land_x_ki", land_.ki.x, 0);
  ros::param::param<double>("behaviors/land_y_ki", land_.ki.y, 0);
  ros::param::param<double>("behaviors/land_z_ki", land_.ki.z, 0);
  ros::param::param<double>("behaviors/land_w_ki", land_.ki.w, 0);

  ros::param::param<double>("behaviors/land_x", land_.land_pose.x, 0);
  ros::param::param<double>("behaviors/land_y", land_.land_pose.y, 0);
  ros::param::param<double>("behaviors/land_z", land_.land_pose.z, 0);
  ros::param::param<double>("behaviors/land_w", land_.land_pose.w, 0);

  // follow
  ros::param::param<double>("behaviors/follow_x_kp", follow_.kp.x, 0);
  ros::param::param<double>("behaviors/follow_y_kp", follow_.kp.y, 0);
  ros::param::param<double>("behaviors/follow_z_kp", follow_.kp.z, 0);
  ros::param::param<double>("behaviors/follow_w_kp", follow_.kp.w, 0);

  ros::param::param<double>("behaviors/follow_x_kd", follow_.kd.x, 0);
  ros::param::param<double>("behaviors/follow_y_kd", follow_.kd.y, 0);
  ros::param::param<double>("behaviors/follow_z_kd", follow_.kd.z, 0);
  ros::param::param<double>("behaviors/follow_w_kd", follow_.kd.w, 0);

  ros::param::param<double>("behaviors/follow_x_ki", follow_.ki.x, 0);
  ros::param::param<double>("behaviors/follow_y_ki", follow_.ki.y, 0);
  ros::param::param<double>("behaviors/follow_z_ki", follow_.ki.z, 0);
  ros::param::param<double>("behaviors/follow_w_ki", follow_.ki.w, 0);

  ros::param::param<double>("behaviors/follow_x", follow_.follow_pose.x, 0);
  ros::param::param<double>("behaviors/follow_y", follow_.follow_pose.y, 0);
  ros::param::param<double>("behaviors/follow_z", follow_.follow_pose.z, 0);
  ros::param::param<double>("behaviors/follow_w", follow_.follow_pose.w, 0);

  // Return
  double settleRadius;
  ros::param::param<double>("behaviors/return_gotoHeight", return_.gotoHeight,
                            5);
  ros::param::param<double>("behaviors/return_finalHeight", return_.finalHeight,
                            3);
  ros::param::param<double>("behaviors/return_downRadius", return_.downRadius,
                            1);
  ros::param::param<double>("behaviors/return_settleRadius", settleRadius, .5);
  ros::param::param<double>("behaviors/return_tagTime", return_.tagTime, 1);
  ros::param::param<double>("behaviors/return_tagLossThresh",
                            return_.tagLossThresh, 5);
  return_.settleRadiusSquared = settleRadius * settleRadius;

  // takeoff
  ros::param::param<double>("behaviors/takeoff_height", takeoff_.height, 0);
  ros::param::param<double>("behaviors/takeoff_threshold", takeoff_.threshold,
                            0);

  // initialize the tag
  tagPose_.pose.orientation.x = 0;
  tagPose_.pose.orientation.y = 0;
  tagPose_.pose.orientation.z = 0;
  tagPose_.pose.orientation.w = 1;

  // Initialize the PID controllers
  xpid_ = new bsc_common::PID(follow_.kp.x, follow_.ki.x, follow_.kd.x);
  ypid_ = new bsc_common::PID(follow_.kp.y, follow_.ki.y, follow_.kd.y);
  zpid_ = new bsc_common::PID(follow_.kp.z, follow_.ki.z, follow_.kd.z);
  wpid_ = new bsc_common::PID(follow_.kp.w, follow_.ki.w, follow_.kd.w);
}

Behaviors::~Behaviors() {}

void Behaviors::doBehaviorAction() {

  simpleTag_.x = tagPose_.pose.position.x;
  simpleTag_.y = tagPose_.pose.position.y;
  simpleTag_.z = tagPose_.pose.position.z;

  simpleTag_.w = bsc_common::util::yaw_from_quat(tagPose_.pose.orientation);

  simpleTag_.t = tagPose_.header.stamp.toSec();

  /*ROS_INFO("x: %1.2f, y:%1.2f, z: %1.2f, yaw: %1.3f",
      simpleTag_.x,
      simpleTag_.y,
      simpleTag_.z,
      simpleTag_.w);*/

  // //Lets grab the most recent time stamp
  // if(uavGPS_.header.stamp.toSec()>boatGPS_.header.stamp.toSec())
  //   simpleTag_.header.stamp = uavGPS_.header.stamp;
  // else
  //   simpleTag_.header.stamp = uavGPS_.header.stamp;

  switch (currentMode_) {
  case JETYAK_UAV::TAKEOFF: {
    takeoffBehavior();
    break;
  }
  case JETYAK_UAV::FOLLOW: {
    followBehavior();
    break;
  }
  case JETYAK_UAV::LEAVE: {
    // Do nothing, an external node is currently communicating with the pilot
    break;
  }
  case JETYAK_UAV::RETURN: {
    returnBehavior();
    break;
  }
  case JETYAK_UAV::LAND: {
    landBehavior();
    break;
  }
  case JETYAK_UAV::RIDE: {
    rideBehavior();
    break;
  }
  case JETYAK_UAV::HOVER: {
    hoverBehavior();
    break;
  }
  default: {
    if (propellorsRunning) {
      ROS_ERROR("Mode out of bounds: %i. Now hovering.", (char)currentMode_);
      this->currentMode_ = JETYAK_UAV::HOVER;
    } else {
      ROS_ERROR("Mode out of bounds: %i. Now riding.", (char)currentMode_);
      this->currentMode_ = JETYAK_UAV::RIDE;
    }
    break;
  }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "behaviors");
  ros::NodeHandle nh;
  Behaviors behaviors_o(nh);
  ros::Rate rate(10);

  while (ros::ok()) {
    ros::spinOnce();

    behaviors_o.doBehaviorAction();

    rate.sleep();
  }
  return 0;
}
