/** Controls the GPS return and landing
* Sends commands to the controller
* sets teh jetyak position as a gps waypoint
* search for the ar tag
  * when found publish approach mode
* uses ar tag to nav around to the landing platform
* Face camera downward and publish landing mode
* land using downward camera
* angle camera up and publish landed mode
*/

#ifndef JETYAK_UAV_UTILS_SEARCH_
#define JETYAK_UAV_UTILS_SEARCH_

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

class search {
  private:
    ros::Subscriber arTagSub_, modeSub_, uavGpsSub_ , uavAttitudeSub_, jetyakGpsSub_, jetyakToPursueSub_;
    ros::Publisher cmdPub_, modePub_, gimbalAnglePub_;

    geometry_msgs::Point lastJetyakGpsCoord_; // x: longitude, y: latitude, z: altitude
    geometry_msgs::Quaternion lastUavGpsCoord_; // x:longitude, y: latitude, z: altitude, w: heading

    double searchVelocity_;
    char currentMode_=0;
    /** arTagCallback
    * If in SEARCHING mode
    *   If the tag is within some threshold of where the jetyak should be, set to APPROACH
    *   Else move toward the jetyak and yaw to face it
    * If in APPROACH
    *   If tag 3 in view, enter follow mode
    *   Else maintain k meter seperation and move left (move CW to find 3)
    *
    * @param msg List of alvar tag info
    */
    void arTagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

    /** mode Callback
    * Set this objects mode
    *
    * @param msg Mode message
    */
    void modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg);

    /** jetyakGpsSub
    * if in SEARCH
    *   record the gps position of jetyak being pursued
    *   send command to controller
    *
    * @param msg Gps coordinates of jetyak being pursued
    */
    void jetyakGpsCallback(const nav_msgs::NavSatFix::ConstPtr& msg);

    /** jetyakToPursue
    * Receive the namespace of the jetyak to pursue, also switches to SEARCHING mode
    *
    * @param msg namespace of the jetyak ex. "jetyak1"
    */
    void jetyakToPursueCallback(const std_msgs::String::ConstPtr& msg);

  public:
    /** Constructor
    * Creates the publishers, subscribers, and service clients
    *
    * @param nh Node handler
    */
    Controller(ros::NodeHandle& nh);

};

#endif
