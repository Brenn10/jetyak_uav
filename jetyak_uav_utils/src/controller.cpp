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

  controlRequestSrv_ = nh.serviceClient<dji_sdk::SDKControlAuthority>("sdk_control_authority");
}

void controller::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {}
void controller::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {
  this->currentMode_ = msg->mode;
  dji_sdk::SDKControlAuthority auth;
  if(this->currentMode_ == msg->AWAY) { //release control
    auth.request.control_enable=0;
    while(!controlRequestSrv_.call(auth))
      ROS_WARN("Unable to release authority");
  }
  else { //check that we have control and/or take control
    auth.request.control_enable=0;
    while(!controlRequestSrv_.call(auth))
      ROS_WARN("Unable to release authority");
  }
}
void controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if(msg->buttons[6] or msg->buttons[7]) //LT or RT (deadswitch)
  {
    command_.priority=JOYSTICKCONTROL;
    if(msg->buttons[1]) {} //land
    else if(msg->buttons[3]) {} //take off
    else { //set vels

      command_.vels.linear.x=-msg->axes[3];
      command_.vels.linear.y=-msg->axes[2];
      command_.vels.linear.z=-msg->axes[1];
      command_.vels.angular.z=-msg->axes[0];
    }
  }
}

void controller::publishCommand() {

  // Hex code says to use horizontal speeds, vertical speeds,
  // and yaw rate relative to the body and active breaking
  std::vector<float> axes {
    (float)command_.vels.linear.y,
    (float)command_.vels.linear.x,
    (float)command_.vels.linear.z,
    (float)command_.vels.angular.z,
    0x4B
  };
  cmdVel_.axes = axes;
  command_.vels =geometry_msgs::Twist(); //reset command
  command_.priority = NOINPUT;
  cmdPub_.publish(cmdVel_);
}

void controller::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

}

int main(int argc, char **argv) {
  ros::init(argc,argv,"uav_controller");
  ros::NodeHandle nh;
  controller uav_controller(nh);

  ros::Rate rate(10);

  while(ros::ok())
  {
    ros::spinOnce();

    uav_controller.publishCommand();

    rate.sleep();
  }
  return 0;
}
