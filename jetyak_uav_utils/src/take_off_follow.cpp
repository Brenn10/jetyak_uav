#include "jetyak_uav_utils/take_off_follow.h"

take_off_follow::take_off_follow(ros::NodeHandle& nh)
{
  // Setup dynamic reconfigure
  f = boost::bind(&reconfigureCallback,_1,_2);
  server.setCallback(f);

  takeoffPub_ = nh.advertise<std_msgs::Empty>("takeoff",1);
  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&take_off_follow::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&take_off_follow::modeCallback,this);
}

void take_off_follow::arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  if(currentMode_==jetyak_uav_utils::Mode::FOLLOWING) {
    geometry_msgs::Quaternion state;
    if(!msg->markers.empty()){
      double currentTime=ros::Time::now().toSec();

      // Get drone last_cmd_update_
      
      if(wasLastLanded_)
      {
        wasLastLanded_=false;
        droneLastSeen=currentTime;
        xpid_=new PID(kp.x,ki.x,kd.x);
        ypid_=new PID(kp.y,ki.y,kd.y);
        zpid_=new PID(kp.z,ki.z,kd.z);
        wpid_=new PID(kp.w,ki.w,kd.w);
        xpid_
        takeoffPub_.publish(std_msgs::Empty);
      }
      else {
        timeDelta = currentTime-droneLastSeen;
        droneLastSeen = currentTime;

        cmdVel.publish(cmdT);
      }
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
