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
    geometry_msgs::Quaternion followPose_;

    char currentMode_=0;

    bool wasLastLanded_=true;

    /** arTagCallback
    * Use tf from jetyak to tags and info on tags to determine position relative to kayak
    * If wasLastLanded_
    *   Set vertical velocity to K m/s to get off the platform.
    *   reset pid controllers
    *   set wasLastLanded_ to false
    * else
    *   PID to follow position
    * if position is within a meter and state is TAKINGOFF, set mode to FOLLOWING
    *
    * @param msg vector of marker info
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    /** modeCallback
    * receives the mode change
    * if state changed to TAKINGOFF, set wasLastLanded_ to true
    *
    * @param msg Mode that has been activated
    */
    void modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg);


  public:
    /** Constructor
    * Creates the publishers, subscribers, and service clients
    *
    * @param nh Node handler
    */
    take_off_follow(ros::NodeHandle& nh);
};
#endif
