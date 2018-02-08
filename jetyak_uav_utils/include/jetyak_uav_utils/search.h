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

#include <ros/ros.h>

class search_and_land {
  private:
    ros::Subscriber arTagSub, modeSub, uavGpsSub ,jetyakGpsSub, gimbalAngleSub;
    ros::Publisher cmdPub, modePub, gimbalAnglePub;
    ros::ServiceClient uploadMissionWaypointSrvC, waypointActionSrvC;
  public:
    Controller(ros::NodeHandle& nh);
    void arTagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
    void modeCallback(const std_msgs::Int8::ConstPtr& msg);
    void uavGpsCallback(const nav_msgs::NavSatFix::ConstPtr& msg);
    void jetyakGpsCallback(const nav_msgs::NavSatFix::ConstPtr& msg);
    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

};

#endif
