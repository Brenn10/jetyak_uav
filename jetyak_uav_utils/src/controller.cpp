#include "jetyak_uav_utils/controller.h"

controller::controller(ros::NodeHandle& nh) {
  ros::param::get("~arTagSafetyDistance",arTagSafetyDistance_);
  ros::param::get("~maxSpeed",maxSpeed_);

  joySub_ = nh.subscribe("joy",1,&controller::joyCallback,this);
  arTagSub_ = nh.subscribe("ar_track_alvar",1,&controller::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&controller::modeCallback,this);
  cmdSub_ = nh.subscribe("raw_cmd",1,&controller::cmdCallback,this);

  cmdPub_ = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  controlRequestSrv_ = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  taskSrv_ = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  activateSrv_ = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");

  currentMode_ = 0;
  dji_sdk::Activation actSrvMsg;
  ROS_WARN("Activation Response: %s",activateSrv_.call(actSrvMsg)? "Success": "Fail");
}

controller::~controller() {
  dji_sdk::SDKControlAuthority srv;
  srv.request.control_enable=0;
  ROS_WARN("Release Response: %s",controlRequestSrv_.call(srv)? "Success": "Fail");
}

void controller::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {}

void controller::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {
  this->currentMode_ = msg->mode;
}
void controller::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  ROS_ERROR("%f",msg->axes[4]);
  if(msg->buttons[6] or msg->buttons[7]) //LT or RT (deadswitch)
  {
    command_.priority=JOYSTICKCONTROL;

    if(msg->buttons[2]) { //give control
      dji_sdk::SDKControlAuthority srv;
      srv.request.control_enable=0;
      ROS_WARN("Release Response: %s",controlRequestSrv_.call(srv)? "Success": "Fail");
    }
    else if(msg->buttons[0]){ //take control
      dji_sdk::SDKControlAuthority srv;
      srv.request.control_enable=1;
      ROS_WARN("Release Response: %s",controlRequestSrv_.call(srv)? "Success": "Fail");
    }
    else if(msg->buttons[1]) { //land
      dji_sdk::DroneTaskControl srv;
      srv.request.task=6;
      ROS_WARN("Release Response: %s",taskSrv_.call(srv)? "Success": "Fail");
    }
    else if(msg->buttons[3]) { //take off
      dji_sdk::DroneTaskControl srv;
      srv.request.task=4;
      ROS_WARN("Release Response: %s",taskSrv_.call(srv)? "Success": "Fail");
    }
    else { //set vels

      command_.vels.linear.x=-msg->axes[3];
      command_.vels.linear.y=-msg->axes[2];
      command_.vels.linear.z=-msg->axes[1];
      command_.vels.angular.z=-msg->axes[0];
    }
  }
}

void controller::publishCommand() {

  //{Left,Forward,Up,CCW}

  cmdVel_.axes = {
    (float)command_.vels.linear.y,
    (float)command_.vels.linear.x,
    (float)command_.vels.linear.z,
    (float)command_.vels.angular.z
  };

  command_.vels = geometry_msgs::Twist(); //reset command
  command_.priority = NOINPUT;
  cmdPub_.publish(cmdVel_);
}

void controller::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if(command_.priority>=EXTERNAL) // If this has a stronger or equal priority, overwrite
  {
    command_.priority=EXTERNAL;
    command_.vels.linear.x=msg->linear.x;
    command_.vels.linear.y=msg->linear.y;
    command_.vels.linear.z=msg->linear.z;
    command_.vels.angular.z=msg->angular.z;
  }
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
