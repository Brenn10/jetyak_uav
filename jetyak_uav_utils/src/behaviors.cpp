#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
 {
  //subscribers
  tagPoseSub_ = nh.subscribe("/jetyak_uav_utils/tag_pose",1,&behaviors::tagPoseCallback, this);
  uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position",1,&behaviors::uavGPSCallback, this);
  boatGPSSub_ = nh.subscribe("boat_gps",1,&behaviors::uavGPSCallback, this);
  uavAttSub_ =  nh.subscribe("/dji_sdk/attitude",1,&behaviors::uavAttitudeCallback, this);
  boatIMUSub_ =  nh.subscribe("boat_imu",1,&behaviors::boatHeadingCallback, this);

  //Publishers
  cmdPub_ = nh.advertise<sensor_msgs::Joy>("jetyak_uav_utils/behavior_cmd",1);

  //service clients
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

  // Service servers
  modeService_ = nh.advertiseService("jetyak_uav_utils/mode", &behaviors::modeCallback,this);
}

behaviors::~behaviors() {}


bool behaviors::modeCallback(jetyak_uav_utils::Mode::Request  &req,
                              jetyak_uav_utils::Mode::Response &res) {

  this->currentMode_ = req.mode;

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

void behaviors::tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  
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
