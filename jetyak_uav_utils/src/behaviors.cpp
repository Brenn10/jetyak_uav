#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
 {
  //subscribers
  tagPoseSub_ = nh.subscribe("/jetyak_uav_utils/tag_pose",1,
    &behaviors::tagPoseCallback, this);
  uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position",1,
    &behaviors::uavGPSCallback, this);
  boatGPSSub_ = nh.subscribe("boat_gps",1,&behaviors::uavGPSCallback, this);
  uavAttSub_ =  nh.subscribe("/dji_sdk/attitude",1,
    &behaviors::uavAttitudeCallback, this);
  boatIMUSub_ =  nh.subscribe("boat_imu",1,
    &behaviors::boatHeadingCallback, this);

  //Publishers
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("jetyak_uav_utils/behavior_cmd",1);

  //service clients
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>(
    "/dji_sdk/drone_task_control");

  // Service servers
  modeService_ = nh.advertiseService("jetyak_uav_utils/mode",
    &behaviors::modeCallback,this);
}

behaviors::~behaviors() {}


bool behaviors::modeCallback(jetyak_uav_utils::Mode::Request  &req,
                              jetyak_uav_utils::Mode::Response &res) {

  this->currentMode_ = req.mode;
  this->behaviorChanged_=true;
  switch(this->currentMode_)
  {
    case req.TAKEOFF: {
      dji_sdk::DroneTaskControl srv;
      srv.request.task=4;
      taskSrv_.call(srv);
      res.success=srv.response.result;
      if(res.success)
      {
        this->currentMode_=req.FOLLOW;
      }
      else
      {
        this->currentMode_=req.HOVER;
      }
    }
    case req.LAND: {}
    case req.FOLLOW: {}
    case req.LEAVE: {}
    case req.RETURN: {
      res.success=true;
      break;
    }

  }
  behaviorChanged=true;
  return res.success;
}

void behaviors::tagPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  tagPose_.pose=msg->pose;
  tagPose_.header=msg->header;
}

void behaviors::uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    uavGPS_.header=msg->header;
    uavGPS_.status=msg->status;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.latitude=msg->latitude;
    uavGPS_.position_covariance=msg->position_covariance;
    uavGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("UAV GNSS fix failed. Status: %s",msg->status.status)
  }
}

void behaviors::boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if(msg->status.status>=0) {
    boatGPS_.header=msg->header;
    boatGPS_.status=msg->status;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.latitude=msg->latitude;
    boatGPS_.position_covariance=msg->position_covariance;
    boatGPS_.position_covariance_type=msg->position_covariance_type;
  } else {
    ROS_WARN("Boat GNSS fix failed. Status: %s",msg->status.status)
  }
}

void behaviors::uavAttitudeCallback(
    const sensor_msgs::QuaternionStamped::ConstPtr& msg) {
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
  dji_sdk::DroneTaskControl srv;
  srv.request.task=4;
  taskSrv_.call(srv);
  if(srv.response) {
    currentMode_=jetyak_uav_utils::Mode.request.FOLLOW;
  }
}

void behaviors::followBehavior() {
  if(behaviorChanged_) {
    xpid_->reset();
    ypid_->reset();
    zpid_->reset();
    wpid_->reset();

    xpid_->updateParams(follow_.kp.x ,follow_.ki.x,follow_.kd.x);
    ypid_->updateParams(follow_.kp.y ,follow_.ki.y,follow_.kd.y);
    zpid_->updateParams(follow_.kp.z ,follow_.ki.z,follow_.kd.z);
    wpid_->updateParams(follow_.kp.w ,follow_.ki.w,follow_.kd.w);
  }
}

void behaviors::doBehaviorAction() {
  switch(currentMode_) {
    case jetyak_uav_utils::Mode.request.TAKEOFF: {
      takeoffBehavior();
      break;
    }
    case jetyak_uav_utils::Mode.request.FOLLOW: {
      followBehavior();
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_behaviors");
  ros::NodeHandle nh;
  behaviors uav_behaviors(nh);
  ros::Rate rate(10);

  while(ros::ok())
  {
    ros::spinOnce();

    uav_behaviors.doBehaviorAction();

    rate.sleep();
  }
  return 0;
}
