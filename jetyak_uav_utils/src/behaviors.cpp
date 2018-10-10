#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
{
  //initialize mode
  currentMode_=Mode::RIDE;

  //subscribers
  tagPoseSub_ = nh.subscribe("tag_pose",1,&behaviors::tagPoseCallback, this);
  uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position",1,&behaviors::uavGPSCallback, this);
  boatGPSSub_ = nh.subscribe("boat_gps",1,&behaviors::boatGPSCallback, this);
  uavAttSub_ =  nh.subscribe("/dji_sdk/attitude",1, &behaviors::uavAttitudeCallback, this);
  boatIMUSub_ =  nh.subscribe("boat_imu",1, &behaviors::boatIMUCallback, this);

  //Publishers
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("behavior_cmd",1);

  //service clients
  armSrv_ = nh.serviceClient<dji_sdk::DroneArmControl>("/dji_sdk/drone_arm_control");

  // Service servers
  setModeService_ = nh.advertiseService("setMode",&behaviors::setModeCallback,this);
  getModeService_ = nh.advertiseService("getMode",&behaviors::getModeCallback,this);
}

behaviors::~behaviors() {}


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

void behaviors::takeoffBehavior() {
  if(!propellorsRunning) {
    dji_sdk::DroneArmControl srv;
    srv.request.arm=1;
    armSrv_.call(srv);
    if(srv.response.result) {
      ROS_WARN("ARMS ENABLED");
      behaviorChanged_=true;
      currentMode_=Mode::FOLLOW;
      propellorsRunning=true;
    } else {
      ROS_WARN("FAILED TO ENABLE ARMS");
    }
  }
  else {
    ROS_WARN("ARMS ALREADY GOING");
    currentMode_=Mode::FOLLOW;
  }
}

void behaviors::followBehavior() {
  if(behaviorChanged_) {
    behaviorChanged_=false;
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();

    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
  } else {

    // line up with pad
    xpid_->update(follow_.follow_pose.x-actualPose_.quaternion.x,actualPose_.header.stamp.toSec());
    ypid_->update(follow_.follow_pose.y-actualPose_.quaternion.y,actualPose_.header.stamp.toSec());
    zpid_->update(follow_.follow_pose.z-actualPose_.quaternion.z,actualPose_.header.stamp.toSec());

    //point at tag
    wpid_->update(follow_.follow_pose.w-actualPose_.quaternion.w,actualPose_.header.stamp.toSec());

    //rotate velocities in reference to the tag
    double rotated_x;
    double rotated_y;
    bsc_common::util::rotate_vector(
      xpid_->get_signal(),ypid_->get_signal(),-actualPose_.quaternion.w,rotated_x,rotated_y);

    sensor_msgs::Joy cmd;
    cmd.axes.push_back(xpid_->get_signal());
    cmd.axes.push_back(ypid_->get_signal());
    cmd.axes.push_back(zpid_->get_signal());
    cmd.axes.push_back(wpid_->get_signal());
    cmd.axes.push_back(bodyVelCmdFlag_);
    cmdPub_.publish(cmd);
  }
}

void behaviors::returnBehavior() {};

void behaviors::landBehavior() {};

void behaviors::rideBehavior() {
  if(propellorsRunning) {
    dji_sdk::DroneArmControl srv;
    srv.request.arm=0;
    armSrv_.call(srv);
    propellorsRunning=srv.response.result;
    if(srv.response.result) {
      ROS_WARN("Arms deactivated");
    } else {
      ROS_WARN("Failed to deactivate arms");
    }
  }
}


void behaviors::hoverBehavior() {
  sensor_msgs::Joy cmd;
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(0);
  cmd.axes.push_back(bodyVelCmdFlag_);
  cmdPub_.publish(cmd);
};

void behaviors::doBehaviorAction() {

  /*
   * Find the UAV pose from the boat
  */
  //compute relative uav heading
  double boatHeading=bsc_common::util::yaw_from_quat(boatImu_.orientation);
  double uavHeading=bsc_common::util::yaw_from_quat(uavImu_.orientation);
  //compute relative uav position
  actualPose_.quaternion.x=uavGPS_.latitude-boatGPS_.latitude;
  actualPose_.quaternion.y=uavGPS_.longitude-boatGPS_.longitude;
  actualPose_.quaternion.z=uavGPS_.altitude-boatGPS_.altitude;
  actualPose_.quaternion.w=bsc_common::util::ang_dist(boatHeading,uavHeading);

  //Lets grab the most recent time stamp
  if(uavGPS_.header.stamp.toSec()>boatGPS_.header.stamp.toSec())
    actualPose_.header.stamp = uavGPS_.header.stamp;
  else
    actualPose_.header.stamp = uavGPS_.header.stamp;

  switch(currentMode_) {
    case Mode::TAKEOFF: {
      takeoffBehavior();
      break;
    }
    case Mode::FOLLOW: {
      followBehavior();
      break;
    }
    case Mode::LEAVE: {
      //Do nothing, an external node is currently communicating with the pilot
      break;
    }
    case Mode::RETURN: {
      returnBehavior();
      break;
    }
    case Mode::LAND: {
      landBehavior();
      break;
    }
    case Mode::RIDE: {
      rideBehavior();
      break;
    }
    case Mode::HOVER: {
      hoverBehavior();
      break;
    }
    default: {
      if(propellorsRunning) {
        ROS_ERROR("Mode out of bounds: %i. Now hovering.",(char)currentMode_);
        this->currentMode_=Mode::HOVER;
      }
      else {
        ROS_ERROR("Mode out of bounds: %i. Now riding.",(char)currentMode_);
        this->currentMode_=Mode::RIDE;
      }
      break;
    }
  }
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

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_behaviors");
  ros::NodeHandle nh;
  behaviors uav_behaviors(nh);
  ros::Rate rate(10);


  //Create Dynamic reconfiguration node handles
  ros::NodeHandle nhl("~/land");
  ros::NodeHandle nhf("~/follow");

  //Create synamic Reconfigure
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig> landCfgServer(nhl);
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig> followCfgServer(nhf);

  // define callback types
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig>::CallbackType landCfgCallback;
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig>::CallbackType followCfgCallback;

  // bind callbacks
  boost::function<void (jetyak_uav_utils::LandConstantsConfig &,int) >
      landCfgCallback2(boost::bind( &behaviors::landReconfigureCallback,&uav_behaviors, _1, _2 ) );
  boost::function<void (jetyak_uav_utils::FollowConstantsConfig &,int) >
      followCfgCallback2(boost::bind( &behaviors::followReconfigureCallback,&uav_behaviors, _1, _2 ) );

  landCfgCallback=landCfgCallback2;
  followCfgCallback=followCfgCallback2;

  //set callbacks in server
  landCfgServer.setCallback(landCfgCallback);
  followCfgServer.setCallback(followCfgCallback);

  while(ros::ok())
  {
    ros::spinOnce();

    uav_behaviors.doBehaviorAction();

    rate.sleep();
  }
  return 0;
}
