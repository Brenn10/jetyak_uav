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

#include "jetyak_uav_utils/Mode.h"

#include <cstdlib>


#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

class controller {
  private:
    ros::Subscriber joySub_, arTagSub_, modeSub_, cmdSub_;
    ros::Publisher cmdPub_, modePub_;
    unsigned char currentMode_;

    // Levels of control priority, lower is stronger
    enum ControllerPriorityLevels {ObstacleAvoidance, JoystickControl, Package, External};
    ControllerPriorityLevels control_priority_;

    double arTagSafetyDistance_, maxSpeed_;


    /** arTagCallback
    * Safety controller that activates if the quad gets too close to an ar tag.
    * Moves away from the tag if in certain modes
    *
    * @param msg List of ar tags detected in frame
    */
    void arTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    /**
    * Changes the current mode
    * changes control_priority
    *
    * @param msg gets the mode from the broadcast
    */
    void modeCallback(const std_msgs::Int8::ConstPtr& msg);

    /** joyCallback
    * http://wiki.ros.org/dji_sdk#Details_on_flight_control_setpoint
    * 0x4B as control flag (Horizontal velocity, vertical velocity, yaw velocity, body ENU frame, active break)
    * RT OR LT: deadswitch
    * Y: takeoff
    * left stick Y:
    */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    /** cmdCallback
    * Passes along a command if deemed safe by safety controller
    *
    * @param msg Twist message of the requested velocities
    */
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

  public:
    /** Constructor
    * Start up the Controller Node
    * Create publishers, subscribers, and request access
    *
    * @param nh node handler
    */
    controller(ros::NodeHandle& nh);
};

#endif
