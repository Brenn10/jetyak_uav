#include "jetyak_uav_utils/land.h"

land::land(ros::NodeHandle& nh)
{
  flyPose_.x=initialFlyPose_.x=5;
  flyPose_.y=initialFlyPose_.y=0;
  flyPose_.z=initialFlyPose_.z=4;
  flyPose_.w=initialFlyPose_.w=0;

  landPub_ = nh.advertise<std_msgs::Empty>("land",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&land::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&land::modeCallback,this);
}

void land::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {


  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING) {
    if(!msg->markers.empty()){
      droneLastSeen_=ros::Time::now().toSec();
      geometry_msgs::Quaternion* state;
      const geometry_msgs::Pose* pose = const_cast<const geometry_msgs::Pose*>(&msg->markers[0].pose.pose);
      bsc_common::util::xyzw_from_pose(pose,state);

      // Get drone last_cmd_update_

      if(firstLandLoop_)
      {
        flyPose_.x=initialFlyPose_.x;
        flyPose_.y=initialFlyPose_.y;
        flyPose_.z=initialFlyPose_.z;
        flyPose_.w=initialFlyPose_.w;

        firstLandLoop_=false;
        xpid_=new bsc_common::PID(kp_.x,ki_.x,kd_.x);
        ypid_=new bsc_common::PID(kp_.y,ki_.y,kd_.y);
        zpid_=new bsc_common::PID(kp_.z,ki_.z,kd_.z);
        wpid_=new bsc_common::PID(kp_.w,ki_.w,kd_.w);

        landPub_.publish(std_msgs::Empty());
      }
      else {
        xpid_->update(flyPose_.x-state->x);
        ypid_->update(flyPose_.y-state->y);
        zpid_->update(flyPose_.z-state->z);
        wpid_->update(flyPose_.w-state->w);


        geometry_msgs::Twist cmdT;
        cmdT.linear.x=xpid_->get_signal();
        cmdT.linear.x=ypid_->get_signal();
        cmdT.linear.x=zpid_->get_signal();
        cmdT.angular.z=wpid_->get_signal();
        cmdT.angular.y=cmdT.angular.x=0;
        cmdPub_.publish(cmdT);
      }
    }
    else { //if not seen in more than a sec, stop and spin. after 5, search
      if(ros::Time::now().toSec()-droneLastSeen_>5) { // if not seen in 5 sec
        jetyak_uav_utils::Mode m;
        m.mode=jetyak_uav_utils::Mode::SEARCHING;
        modePub_.publish(m);
      }
      else if(ros::Time::now().toSec()-droneLastSeen_>1) { //if not seen in 1 sec
        geometry_msgs::Twist cmdT;
        cmdT.linear.x=0;
        cmdT.linear.x=0;
        cmdT.linear.x=0;
        cmdT.angular.z=1.5;
        cmdT.angular.y=cmdT.angular.x=0;
        cmdPub_.publish(cmdT);
      }
    }
  }
}
void land::modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg) {
  currentMode_=msg->mode;
  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING)
  {
    firstLandLoop_=true;
    droneLastSeen_=0;
  }
  else
  {
    droneLastSeen_=0;
  }
}

void land::reconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level) {
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

  initialFlyPose_.x=config.follow_x;
  initialFlyPose_.y=config.follow_y;
  initialFlyPose_.z=config.follow_z;
  initialFlyPose_.w=config.follow_w;

  xpid_->updateParams(kp_.x,ki_.x,kd_.x);
  ypid_->updateParams(kp_.y,ki_.y,kd_.y);
  zpid_->updateParams(kp_.z,ki_.z,kd_.z);
  wpid_->updateParams(kp_.w,ki_.w,kd_.w);
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"land");
  ros::NodeHandle nh;
  land land(nh);
  //Dynamic reconfigure
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig> server;
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig>::CallbackType f;

  boost::function<void (jetyak_uav_utils::FollowConstantsConfig &,int) > f2( boost::bind( &land::reconfigureCallback,&land, _1, _2 ) );

  // (iv) Set the callback to the service server.
  f=f2; // Copy the functor data f2 to our dynamic_reconfigure::Server callback type
  server.setCallback(f);
  ros::spin();
  return 0;
}
