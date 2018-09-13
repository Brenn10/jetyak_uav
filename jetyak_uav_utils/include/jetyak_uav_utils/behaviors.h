/** Base controller for the uav
* Manages the high level methods of the system.
* Sends commands to the UAV
* implements a joystick ovveride
* implements a safety controller for if the drone is too close to an object
* Listens to topic to determine when to switch modes
*/

#ifndef JETYAK_UAV_UTILS_BEHAVIORS_H_
#define JETYAK_UAV_UTILS_BEHAVIORS_H_

#include "ros/ros.h"

#include "jetyak_uav_utils/Mode.h"
#include "../lib/bsc_common/include/util.h"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <dji_sdk/DroneTaskControl.h>

#include <cstdlib>
#include <vector>



#include "sensor_msgs/Joy.h"
#include "../lib/bsc_common/include/pid.h"
#include "../lib/bsc_common/include/util.h"

class behaviors {
  private:
    bsc_common::PID *xpid_,*ypid_,*zpid_,*wpid_;
    geometry_msgs::Quaternion uavPose_; // holds xyz position and w yaw

    ros::Subscriber arTagSub_;
    ros::Publisher cmdPub_;
    ros::ServiceClient taskSrv_;
    ros::ServiceServer modeService_;
    bool behaviorChanged=false;
    char currentMode_;
    double tagLastSeen_;


    // Land specific constants
    struct {
      geometry_msgs::Quaternion kp,kd,ki;
      const double START_HEIGHT=3.0;
      double currGoalHeight;
      double collapseRatio;

    } land_;

    /**
    * Changes the current mode
    * changes control_priority
    *
    * @param msg gets the mode from the broadcast
    */
    bool modeCallback(jetyak_uav_utils::Mode::Request  &req,
                      jetyak_uav_utils::Mode::Response &res);
    /**
    * Changes the current mode
    * changes control_priority
    *
    * @param msg gets the mode from the broadcast
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

  public:
    /** Constructor
    * Start up the Controller Node
    * Create publishers, subscribers, and request access
    *
    * @param nh node handler
    */
    behaviors(ros::NodeHandle& nh);

    ~behaviors();
    /** publishCommand
    * Calls the cmdPub on the highest priority command passed in
    */
};

#endif
