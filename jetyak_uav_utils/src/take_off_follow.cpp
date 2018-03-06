#include "jetyak_uav_utils/take_off_follow.h"

take_off_follow::take_off_follow(ros::NodeHandle& nh)
{
  ros::param::get("~followx",followPose_.x);
  ros::param::get("~followy",followPose_.y);
  ros::param::get("~followz",followPose_.z);
  ros::param::get("~followyaw",followPose_.w);

  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&take_off_follow::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&take_off_follow::modeCallback,this);
}

void take_off_follow::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if(!msg->markers.empty()){
    if(wasLastLanded_)
    {
      geometry_msgs::Twist cmd;
      cmd.linear.z=1;
      cmd.linear.x=cmd.linear.y=cmd.angular.x=cmd.angular.y=cmd.angular.z=0;
      wasLastLanded_=false;
      cmdPub_.publish(cmd);
    }
    else {

    }
  }
}
void take_off_follow::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"take_off_follow");
  ros::NodeHandle nh;
  take_off_follow takeOffFollow(nh);
  ros::spin();
  return 0;
}
