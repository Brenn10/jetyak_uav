#include "jetyak_uav_utils/search.h"

search::search(ros::NodeHandle& nh)
{
  flyPose_.x=initialFlyPose_.x=5;
  flyPose_.y=initialFlyPose_.y=0;
  flyPose_.z=initialFlyPose_.z=4;
  flyPose_.w=initialFlyPose_.w=0;

  cmdPub_ = nh.advertise<geometry_msgs::Twist>("raw_cmd",1);
  modePub_ = nh.advertise<jetyak_uav_utils::Mode>("uav_mode",1);

  arTagSub_ = nh.subscribe("ar_track_alvar",1,&search::arTagCallback, this);
  modeSub_ = nh.subscribe("uav_mode",1,&search::modeCallback,this);
}

void search::arTagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  Mode newmode;
  newmode.mode=jetyak_uav_utils::Mode::FOLLOWING;
  modePub_.publish(newmode)
}


int main(int argc, char *argv[]) {
  ros::init(argc,argv,"search");
  ros::NodeHandle nh;
  search search(nh);
  //Dynamic reconfigure
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig> server;
  dynamic_reconfigure::Server<jetyak_uav_utils::FollowConstantsConfig>::CallbackType f;

  boost::function<void (jetyak_uav_utils::FollowConstantsConfig &,int) > f2( boost::bind( &search::reconfigureCallback,&search, _1, _2 ) );

  // (iv) Set the callback to the service server.
  f=f2; // Copy the functor data f2 to our dynamic_reconfigure::Server callback type
  server.setCallback(f);
  ros::spin();
  return 0;
}
