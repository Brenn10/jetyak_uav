#include "jetyak_uav_utils/take_off_follow.h"

take_off_follow::take_off_follow(ros::NodeHandle& nh)
{

  takeoffPub_ = nh.advertise<std_msgs::Empty>("takeoff",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&take_off_follow::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&take_off_follow::modeCallback,this);
}

void take_off_follow::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if(!msg->markers.empty()){
    if(wasLastLanded_)
    {
<<<<<<< HEAD
      takeoffPub_.publish(std_msgs::Empty);
=======
      geometry_msgs::Twist cmd;
      cmd.linear.z=1;
      cmd.linear.x=cmd.linear.y=cmd.angular.x=cmd.angular.y=cmd.angular.z=0;
      wasLastLanded_=false;
      cmdPub_.publish(cmd);
>>>>>>> 06e4087652ece293b2c06eaf5f2746b9b33007f7
    }
    else {

    }
  }
}
void take_off_follow::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {

}

void take_off_follow::reconfigureCallback(jetyak_uav_utils::follow_constants &config, uint32_t level) {
  kp.x=config.kp_x;
  kp.y=config.kp_y;
  kp.z=config.kp_z;
  kp.w=config.kp.w;

  kd.x=config.kd_x;
  kd.y=config.kd_y;
  kd.z=config.kd_z;
  kd.w=config.kd.w;

  ki.x=config.ki_x;
  ki.y=config.ki_y;
  ki.z=config.ki_z;
  ki.w=config.ki.w;
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"take_off_follow");
  ros::NodeHandle nh;
  take_off_follow takeOffFollow(nh);
  ros::spin();
  return 0;
}
