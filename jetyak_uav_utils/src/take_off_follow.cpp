#include "jetyak_uav_utils/take_off_follow.h"

take_off_follow::take_off_follow(ros::NodeHandle& nh)
{
  // Setup dynamic reconfigure
  f = boost::bind(&take_off_follow::reconfigureCallback,_1,_2);
  server.setCallback(f);

  takeoffPub_ = nh.advertise<std_msgs::Empty>("takeoff",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&take_off_follow::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&take_off_follow::modeCallback,this);
}

void take_off_follow::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING) {
    if(!msg->markers.empty()){
      geometry_msgs::Quaternion* state;
      bsc_common::util::xyzw_from_pose(msg->markers[0].pose.pose,state);

      // Get drone last_cmd_update_

      if(wasLastLanded_)
      {
        wasLastLanded_=false;
        xpid_=new bsc_common::PID(kp_.x,ki_.x,kd_.x);
        ypid_=new bsc_common::PID(kp_.y,ki_.y,kd_.y);
        zpid_=new bsc_common::PID(kp_.z,ki_.z,kd_.z);
        wpid_=new bsc_common::PID(kp_.w,ki_.w,kd_.w);

        takeoffPub_.publish(bsc_common::util::empty);
      }
      else {
        xpid_->update(follow_pos_.x-state->x);
        ypid_->update(follow_pos_.y-state->y);
        zpid_->update(follow_pos_.z-state->z);
        wpid_->update(follow_pos_.w-state->w);


        geometry_msgs::Twist cmdT;
        cmdT.linear.x=xpid_->get_signal();
        cmdT.linear.x=ypid_->get_signal();
        cmdT.linear.x=zpid_->get_signal();
        cmdT.angular.z=wpid_->get_signal();
        cmdT.angular.y=cmdT.angular.x=0;
        cmdVel.publish(cmdT);
      }
    }
  }
}
void take_off_follow::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {

}

void take_off_follow::reconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level) {
  kp_.x=config.kp_x;
  kp_.y=config.kp_y;
  kp_.z=config.kp_z;
  kp_.w=config.kp_w;

  kd_.x=config.kd_x;
  kd_.y=config.kd_y;
  kd_.z=config.kd_z;
  kd_.w=config.kd_w;

  ki_.x=config.ki_x;
  ki_.y=config.ki_y;
  ki_.z=config.ki_z;
  ki_.w=config.ki_w;


}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"take_off_follow");
  ros::NodeHandle nh;
  take_off_follow takeOffFollow(nh);
  ros::spin();
  return 0;
}
