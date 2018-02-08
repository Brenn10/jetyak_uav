/** Controls taking off and following modes
* Angles camera down on liftoff and uses that camera to move up and back
* Angles camera up to find tag2 and follow at some xyz,yaw
* Keeps tag in that position
*/

#ifndef JETYAK_UAV_UTILS_TAKE_OFF_FOLLOW_H_
#define JETYAK_UAV_UTILS_TAKE_OFF_FOLLOW_H_

#include "ros/ros.h"

#include "jetyak_uav_utils/Mode.h"


#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

class take_off_follow {
  private:
    ros::Subscriber arTagSub_, modeSub_;
    ros::Publisher cmdPub_, modePub_;

    // x: x dist, y: y dist, z: z dist, w: yaw
    geometry_msgs::Quaternion followPose;

    char currentMode_=0;

    bool wasLastLanded=true;

    /** arTagCallback
    * I will be giving x,y,z,w examples
    * if takeoff
    *   move to -2,0,2,0 relative to tag0 or until tag1 spotted
    *   move to -2,0,2,0 relative to tag1 or until tag2 spotted
    *   move to -2,0,2,0 relative to tag2 or until tag3 spotted
    *   set to following when tag 3 in view
    * if following
    *   pid to followPose
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg);


  public:
    /** Constructor
    * Starts up the node disabled
    * Creates the publishers, subscribers, and service clients
    *
    * @param nh Node handler
    */
    take_off_follow(ros::NodeHandle& nh);
};
#endif
