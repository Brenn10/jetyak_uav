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

#ifndef JETYAK_UAV_UTILS_SEARCH_AND_LAND
#define JETYAK_UAV_UTILS_SEARCH_AND_LAND

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#incluse "std_msgs/String.h"

class search {
  private:
    ros::Subscriber arTagSub_, modeSub_, uavGpsSub_ ,jetyakGpsSub_, jetyakToPursueSub_;
    ros::Publisher cmdPub_, modePub_, gimbalAnglePub_;
    ros::ServiceClient uploadMissionWaypointSrv_, waypointActionSrv_;

    /** arTagCallback
    * If in SEARCHING mode
    *   If the tag is within some threshold of teh gps coordinates of the jetyak being searched for, enter APPROACH.
    *     *
    * @param msg List of alvar tag info
    */
    void arTagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

    /** mode Callback
    * Set this objects mode
    *
    * @param msg Mode message
    */
    void modeCallback(const std_msgs::Int8::ConstPtr& msg);

    /** uavGpsSub
    * record the gps position of this uav and publish a command
    *
    * @param msg Gps coordinates of uav
    */
    void uavGpsCallback(const nav_msgs::NavSatFix::ConstPtr& msg);

    /** jetyakGpsSub
    * record the gps position of jetyak being pursued and publish a command
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
