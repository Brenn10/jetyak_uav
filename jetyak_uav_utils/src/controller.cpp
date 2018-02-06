#include "jetyak_uav_utils/controller.h"

controller::controller(ros::NodeHandle& nh) {
  ros::param::get("~arTagSafetyDistance",arTagSafetyDistance_);
  ros::param::get("~maxSpeed",maxSpeed_);

  joySub_ = nh.subscribe("joy_raw_input",1,&controller::joyCallback,this);
  arTagSub_ = nh.subscribe("ar_track_alvar",1,&controller::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&controller::modeCallback,this);
  cmdSub_ = nh.subscribe("raw_cmd",1,&controller::cmdCallback,this);

  cmdPub_ = nh.advertise<sensor_msgs::Joy>("flight_control_setpoint_generic",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);
}

void controller::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if(!msg->markers.empty())
  {
    for(int i = markers.length )
  }
}
void controller::modeCallback(const std_msgs::Int8::ConstPtr& msg){}
void controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){}
void controller::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg){}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_controller");
  ros::NodeHandle nh;
  controller uav_controller(nh);
  ros::spin();
  return 0;
}
