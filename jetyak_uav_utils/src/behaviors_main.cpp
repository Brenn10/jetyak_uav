#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh) :
  xpid_(),ypid_(),zpid_(),wpid_()
{
  //initialize mode
  currentMode_=Mode::HOVER;

  //subscribers
  tagPoseSub_ = nh.subscribe("/tag_pose",1,&behaviors::tagPoseCallback, this);
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


  /****************************************
  * ASSIGNING ROS PARAMETERS TO THE NODE
  ***************************************/
  //land pid
  ros::param::param<double>("land_x_kp", land_.kp.x, 0);
  ros::param::param<double>("land_y_kp", land_.kp.y, 0);
  ros::param::param<double>("land_z_kp", land_.kp.z, 0);
  ros::param::param<double>("land_w_kp", land_.kp.w, 0);

  ros::param::param<double>("land_x_kd", land_.kd.x, 0);
  ros::param::param<double>("land_y_kd", land_.kd.y, 0);
  ros::param::param<double>("land_z_kd", land_.kd.z, 0);
  ros::param::param<double>("land_w_kd", land_.kd.w, 0);

  ros::param::param<double>("land_x_ki", land_.ki.x, 0);
  ros::param::param<double>("land_y_ki", land_.ki.y, 0);
  ros::param::param<double>("land_z_ki", land_.ki.z, 0);
  ros::param::param<double>("land_w_ki", land_.ki.w, 0);

  ros::param::param<double>("land_collapse", land_.collapseRatio, .99);


  //follow
  ros::param::param<double>("follow_x_kp", follow_.kp.x, 0);
  ros::param::param<double>("follow_y_kp", follow_.kp.y, 0);
  ros::param::param<double>("follow_z_kp", follow_.kp.z, 0);
  ros::param::param<double>("follow_w_kp", follow_.kp.w, 0);

  ros::param::param<double>("follow_x_kd", follow_.kd.x, 0);
  ros::param::param<double>("follow_y_kd", follow_.kd.y, 0);
  ros::param::param<double>("follow_z_kd", follow_.kd.z, 0);
  ros::param::param<double>("follow_w_kd", follow_.kd.w, 0);

  ros::param::param<double>("follow_x_ki", follow_.ki.x, 0);
  ros::param::param<double>("follow_y_ki", follow_.ki.y, 0);
  ros::param::param<double>("follow_z_ki", follow_.ki.z, 0);
  ros::param::param<double>("follow_w_ki", follow_.ki.w, 0);

  ros::param::param<double>("follow_x", follow_.follow_pose.x, 0);
  ros::param::param<double>("follow_y", follow_.follow_pose.y, 0);
  ros::param::param<double>("follow_z", follow_.follow_pose.z, 0);
  ros::param::param<double>("follow_w", follow_.follow_pose.w, 0);

}

behaviors::~behaviors() {}

void behaviors::doBehaviorAction() {


  actualPose_.x=tagPose_.pose.position.x;
  actualPose_.y=tagPose_.pose.position.y;
  actualPose_.z=tagPose_.pose.position.z;

  actualPose_.w=bsc_common::util::yaw_from_quat(tagPose_.pose.orientation);

  actualPose_.t=tagPose_.header.stamp.toSec();

  ROS_INFO("x: %1.2f, y:%1.2f, z: %1.2f, yaw: %1.3f",
      actualPose_.x,
      actualPose_.y,
      actualPose_.z,
      actualPose_.w);

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



  while(ros::ok())
  {
    ros::spinOnce();

    uav_behaviors.doBehaviorAction();

    rate.sleep();
  }
  return 0;
}
