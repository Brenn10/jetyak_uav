#include "jetyak_uav_utils/behaviors.h"

behaviors::behaviors(ros::NodeHandle& nh) {

  cmdPub_ = nh.advertise<sensor_msgs::Joy>("jetyak_uav_utils/behaviorCmd",1);
  modeService_ = nh.advertiseService("jetyak_uav_utils/mode", &behaviors::modeCallback,this);
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

  roll_=pitch_=thrust_=yaw_=0;

  currentMode_ = 0;

}

behaviors::~behaviors() {}

bool behaviors::modeCallback(jetyak_uav_utils::Mode::Request  &req,
                              jetyak_uav_utils::Mode::Response &res) {
  dji_sdk::DroneTaskControl srv;
  this->currentMode_ = req.mode;

  switch(this->currentMode_) {
    case req.TAKEOFF:
      srv.request.task=4;
    case req.LAND: {
      srv.request.task=6;
      taskSrv_.call(srv);
      res.success=srv.response.result;
      break;
    }
    case req.FOLLOW: {}
    case req.LEAVE: {}
    case req.RETURN: {
      res.success=true;
      break;
    }

  }
}



void behaviors::publishCommand() {

}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_behaviors");
  ros::NodeHandle nh;
  behaviors uav_behaviors(nh);

  ros::Rate rate(10);

  while(ros::ok())
  {
    ros::spinOnce();

    uav_behaviors.publishCommand();

    rate.sleep();
  }
  return 0;
}
