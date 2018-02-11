/** Controls the uav as it lands
*/

#ifndef JETYAK_UAV_UTILS_LAND_H_
#define JETYAK_UAV_UTILS_LAND_H_

#include "ros/ros.h"

#include "jetyak_uav_utils/Mode.h"


#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

class land {
  private:
    ros::Subscriber arTagSub_, modeSub_;
    ros::Publisher cmdPub_, modePub_;

    // x: x dist, y: y dist, z: z dist, w: yaw
    geometry_msgs::Quaternion followPose_;

    char currentMode_=0;

    bool wasLastLanded_=true;

    /** arTagCallback
    * If mode is LANDING
    * Use tf from jetyak to tags and info on tags to determine position relative to kayak
    * Slowly move to the landing platform and drop when within threshhold
    * Switch to LANDED when throttle is cut
    *
    * @param msg vector of marker info
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    /** modeCallback
    * receives the mode change
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
    land(ros::NodeHandle& nh);
};
#endif
