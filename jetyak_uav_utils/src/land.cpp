#include "jetyak_uav_utils/land.h"

land::land(ros::NodeHandle& nh):
  xpid_(NULL),
  ypid_(NULL),
  zpid_(NULL),
  wpid_(NULL)
{

  landPub_ = nh.advertise<std_msgs::Empty>("land",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("/ar_pose_marker",1,&land::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&land::modeCallback,this);
}

void land::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {


  if(currentMode_==jetyak_uav_utils::Mode::LANDING) {
    if(!msg->markers.empty()){
      droneLastSeen_=ros::Time::now().toSec();
      geometry_msgs::Vector3* state;

      const geometry_msgs::Quaternion* orientation =
          const_cast<const geometry_msgs::Quaternion*>
          (&msg->markers[0].pose.pose.orientation);

      bsc_common::util::rpy_from_quat(orientation,state);

      // Get drone last_cmd_update_

      if(firstLandLoop_)
      {
        currGoalHeight_=startHeight_;

        firstLandLoop_=false;

        xpid_->reset();
        ypid_->reset();
        zpid_->reset();
        wpid_->reset();

        landPub_.publish(std_msgs::Empty());
      }
      else {
        currGoalHeight_*=collapseRatio_;

        // line up with center of pad
        xpid_->update(padCenter_.x-(-msg->markers[0].pose.pose.position.x));
        ypid_->update(padCenter_.y-(-msg->markers[0].pose.pose.position.z));

        //descend
        zpid_->update(currGoalHeight_-msg->markers[0].pose.pose.position.y);

        //point at tag
        wpid_->update(-state->y);// pitch of tag TODO: Check sign

        //rotate velocities in reference to the tag
        double *rotated_x;
        double *rotated_y;
        bsc_common::util::rotate_vector(
          xpid_->get_signal(),ypid_->get_signal(),state->y,rotated_x,rotated_y);

        geometry_msgs::Twist cmdT;
        cmdT.linear.x=*rotated_x;
        cmdT.linear.y=*rotated_y;
        cmdT.linear.z=zpid_->get_signal();
        cmdT.angular.z=wpid_->get_signal();
        cmdT.angular.y=cmdT.angular.x=0;
        cmdPub_.publish(cmdT);
      }
    }
    else { //if not seen in more than a sec, stop and spin. after 5, search
      if(ros::Time::now().toSec()-droneLastSeen_>5) { // if not seen in 5 sec
        // TODO: Make safer by increasing altitude first
        jetyak_uav_utils::Mode m;
        m.mode=jetyak_uav_utils::Mode::SEARCHING;
        modePub_.publish(m);
      }
      else if(ros::Time::now().toSec()-droneLastSeen_>1) { //if not seen in 1 sec
        geometry_msgs::Twist cmdT;
        cmdT.linear.x=0;
        cmdT.linear.y=0;
        cmdT.linear.z=0;
        cmdT.angular.z=1.5;
        cmdT.angular.y=cmdT.angular.x=0;
        cmdPub_.publish(cmdT);
      }
    }
  } else {
    firstLandLoop_=true;

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

void land::reconfigureCallback(jetyak_uav_utils::LandConstantsConfig &config, uint32_t level) {
  ROS_WARN("%s","Reconfigure received by land");

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

  padCenter_.x=config.pad_center_x;
  padCenter_.y=config.pad_center_y;
  padCenter_.z=config.pad_center_z;

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

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"land");
  ros::NodeHandle nh;
  land land_o(nh);
  //Dynamic reconfigure
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig> server;
  dynamic_reconfigure::Server<jetyak_uav_utils::LandConstantsConfig>::CallbackType f;

  boost::function<void (jetyak_uav_utils::LandConstantsConfig &,int) >
      f2(boost::bind( &land::reconfigureCallback,&land_o, _1, _2 ) );

  // (iv) Set the callback to the service server.
  f=f2; // Copy the functor data f2 to our dynamic_reconfigure::Server callback type
  server.setCallback(f);
  ros::spin();
  return 0;
}
