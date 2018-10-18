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

void behaviors::doBehaviorAction() {


  actualPose_.quaternion.x=tagPose_.pose.position.x;
  actualPose_.quaternion.y=tagPose_.pose.position.y;
  actualPose_.quaternion.z=tagPose_.pose.position.z;

  actualPose_.quaternion.w=bsc_common::util::yaw_from_quat(tagPose_.pose.orientation);
  ROS_INFO("x: %1.2f, y:%1.2f, z: %1.2f, yaw: %1.3f",
      actualPose_.quaternion.x,
      actualPose_.quaternion.y,
      actualPose_.quaternion.z,
      actualPose_.quaternion.w);

  // //
  // // Find the UAV pose from the boat through GPS
  // //
  // //compute relative uav heading
  // double boatHeading=bsc_common::util::yaw_from_quat(boatImu_.orientation);
  // double uavHeading=bsc_common::util::yaw_from_quat(uavImu_.orientation);
  // //compute relative uav position
  // actualPose_.quaternion.x=uavGPS_.latitude-boatGPS_.latitude;
  // actualPose_.quaternion.y=uavGPS_.longitude-boatGPS_.longitude;
  // actualPose_.quaternion.z=uavGPS_.altitude-boatGPS_.altitude;
  // actualPose_.quaternion.w=bsc_common::util::ang_dist(boatHeading,uavHeading);
  //
  // //Lets grab the most recent time stamp
  // if(uavGPS_.header.stamp.toSec()>boatGPS_.header.stamp.toSec())
  //   actualPose_.header.stamp = uavGPS_.header.stamp;
  // else
  //   actualPose_.header.stamp = uavGPS_.header.stamp;

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
