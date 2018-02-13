/** Base controller for the uav
* Manages the high level methods of the system.
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
#include <vector>

#include "dji_sdk/SDKControlAuthority.h"

#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"

class controller {
  private:

    ros::Subscriber joySub_, arTagSub_, modeSub_, cmdSub_;
    ros::Publisher cmdPub_, modePub_;
    ros::ServiceClient controlRequestSrv_;
    char currentMode_;

    sensor_msgs::Joy cmdVel_; //ONLY USE WITH publishCommand METHOD

    // Levels of control priority, lower is stronger
    enum CommandPriorityLevels {JOYSTICKCONTROL, OBSTACLEAVOIDANCE, EXTERNAL, NOINPUT};

    double arTagSafetyDistance_, maxSpeed_;

    //Keeps the command we will use with its priority
    struct CommandAndPriority {
      CommandPriorityLevels priority;
      geometry_msgs::Twist vels;
    } command_;

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
    void modeCallback(const jetyak_uav_utils::Mode::ConstPtr& msg);

    /** joyCallback
    * http://wiki.ros.org/dji_sdk#Details_on_flight_control_setpoint
    * 0x4B as control flag (Horizontal velocity, vertical velocity, yaw velocity, body ENU frame, active break)
    * RT OR LT: deadswitch
    * Y: takeoff
    * left stick Y: z vel
    * left stick X: z ang
    * right stick Y: x vel
    * right stick X: y vel
    *
    * @param msg Joy message from joy node
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

    /** publishCommand
    * Calls the cmdPub on the highest priority command passed in
    */
    void publishCommand();
};

#endif
