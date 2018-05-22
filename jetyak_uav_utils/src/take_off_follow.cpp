#include "jetyak_uav_utils/take_off_follow.h"
#include <iostream>
take_off_follow::take_off_follow(ros::NodeHandle& nh) :
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
{
  takeoffPub_ = nh.advertise<std_msgs::Empty>("takeoff",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd_vel_FLU",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&take_off_follow::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&take_off_follow::modeCallback,this);
}

void take_off_follow::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{

  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING)
  {
    if(!msg->markers.empty())
    {
      droneLastSeen_=ros::Time::now().toSec();
      geometry_msgs::Vector3* state;

      const geometry_msgs::Quaternion* orientation = const_cast<const geometry_msgs::Quaternion*>(&msg->markers[0].pose.pose.orientation);
      bsc_common::util::rpy_from_quat(orientation,state);

      // Get drone last_cmd_update_

      if(firstFollowLoop_)
      {
        firstFollowLoop_=false;
        xpid_->reset();
        ypid_->reset();
        zpid_->reset();
        wpid_->reset();

        takeoffPub_.publish(std_msgs::Empty());
      }
      else
      {
        xpid_->update(follow_pos_.x-(-msg->markers[0].pose.pose.position.x));
        ypid_->update(follow_pos_.y-(-msg->markers[0].pose.pose.position.z));
        zpid_->update(follow_pos_.z-msg->markers[0].pose.pose.position.y);
        wpid_->update(follow_pos_.w-state->y); //pitch of the tag


        geometry_msgs::Twist cmdT;
        cmdT.linear.x=xpid_->get_signal();
        cmdT.linear.y=ypid_->get_signal();
        cmdT.linear.z=zpid_->get_signal();
        cmdT.angular.x = 0;
        cmdT.angular.y = 0;
        cmdT.angular.z=wpid_->get_signal();
        cmdPub_.publish(cmdT);
      }
    }
    else
    { //if not seen in more than a sec, stop and spin. after 5, search
      if(ros::Time::now().toSec()-droneLastSeen_>5)
      { // if not seen in 5 sec
        jetyak_uav_utils::Mode m;
        m.mode=jetyak_uav_utils::Mode::SEARCHING;
        modePub_.publish(m);
      }
      else if(ros::Time::now().toSec()-droneLastSeen_>1)
      { //if not seen in 1 sec
        geometry_msgs::Twist cmdT;
        cmdT.linear.x = 0;
        cmdT.linear.y = 0;
        cmdT.linear.z = 0;
        cmdT.angular.x = 0;
        cmdT.angular.y = 0;
        // TODO: if the gimbal is used rotate camera not drone
        cmdT.angular.z = 1.5;
        cmdPub_.publish(cmdT);
      }
    }
  } else {
    firstFollowLoop_ = true;
  }
}
void take_off_follow::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg)
{
  currentMode_=msg->mode;
  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING)
  {
    firstFollowLoop_=true;
    droneLastSeen_=0;
  }
  else
  {
    droneLastSeen_=0;
  }
}

void take_off_follow::reconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level)
{
  ROS_WARN("%s","Reconfigure received by take_off_follow");
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

  followPose_.x=config.follow_x;
  followPose_.y=config.follow_y;
  followPose_.z=config.follow_z;
  followPose_.w=config.follow_w;

  if (xpid_ != NULL)
  {
    xpid_->updateParams(kp_.x,ki_.x,kd_.x);
    ypid_->updateParams(kp_.y,ki_.y,kd_.y);
    zpid_->updateParams(kp_.z,ki_.z,kd_.z);
    wpid_->updateParams(kp_.w,ki_.w,kd_.w);
  } else {
    xpid_ = new bsc_common::PID(kp_.x,ki_.x,kd_.x);
    ypid_ = new bsc_common::PID(kp_.y,ki_.y,kd_.y);
    zpid_ = new bsc_common::PID(kp_.z,ki_.z,kd_.z);
    wpid_ = new bsc_common::PID(kp_.w,ki_.w,kd_.w);
  }

}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"take_off_follow");
  ros::NodeHandle nh;
  take_off_follow takeOffFollow(nh);

  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig> server;
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig>::CallbackType f;
  boost::function<void (jetyak_uav_utils::FollowConstantsConfig &,int) >
      f2( boost::bind( &take_off_follow::reconfigureCallback,&takeOffFollow, _1, _2 ) );

  f=f2;
  server.setCallback(f);
  ros::spin();
  return 0;
}
