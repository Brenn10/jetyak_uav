/** Base controller for the uav
Manages the high level methods of the system.
* Sends commands to the UAV
* implements a joystick ovveride
* implements a safety controller for if the drone is too close to an object
* Listens to topic to determine when to switch modes
*/

#ifndef JETYAK_UAV_UTILS_CONTROLLER_H_
#define JETYAK_UAV_UTILS_CONTROLLER_H_

#include "ros/ros.h"

class controller {
  private:
    ros::Subscriber joySub, arTagSub, modeSub;
    ros::Publisher cmdPub, modePub;
    Mode currentMode;
  public:

    Controller(ros::NodeHandle& nh);
    void arTagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
    void modeCallback(const std_msgs::Int8::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif
