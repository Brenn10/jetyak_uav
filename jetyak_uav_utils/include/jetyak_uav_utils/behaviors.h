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
#include <dji_sdk/DroneTaskControl.h>

#include <cstdlib>
#include <vector>



#include "sensor_msgs/Joy.h"

class behaviors {
  private:

    ros::Subscriber arTagSub_;
    ros::Publisher cmdPub_;
    ros::ServiceClient taskSrv_;
    ros::ServiceServer modeService_;
    double roll_,pitch_,thrust_,yaw_;
    char currentMode_;


    /**
    * Changes the current mode
    * changes control_priority
    *
    * @param msg gets the mode from the broadcast
    */
    bool modeCallback(jetyak_uav_utils::Mode::Request  &req,
                      jetyak_uav_utils::Mode::Response &res);

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
    void publishCommand();
};

#endif
