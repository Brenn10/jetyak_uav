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
#include "dji_sdk/dji_sdk.h"


//C includes
#include <cstdlib>
#include <vector>

//Jetyak UAV Includes
#include "jetyak_uav_utils/Mode.h"
#include "jetyak_uav_utils/LandConstantsConfig.h"
#include "jetyak_uav_utils/FollowConstantsConfig.h"

//ROS Core includes
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

//ROS Packages includes
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dynamic_reconfigure/server.h>

//Custom Lib includes
#include "../lib/bsc_common/include/pid.h"
#include "../lib/bsc_common/include/util.h"

class behaviors {
private:


  /*********************************************
  * ROS PUBLISHERS, SUBSCRIBERS, AND SERVICES
  *********************************************/
  ros::Subscriber tagPoseSub_, boatGPSSub_, boatIMUSub_, uavGPSSub_, uavAttSub_;
  ros::Publisher cmdPub_;
  ros::ServiceClient armSrv_;
  ros::ServiceServer modeService_;

  /**********************
  * INSTANCE VARIABLES
  **********************/
  bsc_common::PID *xpid_,*ypid_,*zpid_,*wpid_; // pid controllers
  bool behaviorChanged_=false;
  char currentMode_=0;
  bool propellorsRunning=false;
  char commandFlag_ = (
    DJISDK::VERTICAL_VELOCITY   |
    DJISDK::HORIZONTAL_VELOCITY |
    DJISDK::YAW_RATE            |
    DJISDK::HORIZONTAL_BODY     |
    DJISDK::STABLE_ENABLE);

  /************************************
  * STATE VARIABLES
  ************************************/
  sensor_msgs::NavSatFix uavGPS_,boatGPS_;
  geometry_msgs::QuaternionStamped uavAttitude_;
  sensor_msgs::Imu uavImu_, boatImu_;
  geometry_msgs::PoseStamped tagPose_;


  /*********************************************
  * BEHAVIOR SPECIFIC VARIABLES AND CONSTANTS
  **********************************************/
  // Land specific constants
  struct {
    geometry_msgs::Quaternion kp,kd,ki;
    double currGoalHeight;
    double collapseRatio;

  } land_;

  // follow specific constants
  struct {
    geometry_msgs::Quaternion kp,kd,ki;
    geometry_msgs::Quaternion follow_pose;
  } follow_;

  /*********************
  * SERVICE CALLBACKS
  *********************/
  /** modeCallback
  * Callback for the mode service. Changes the current mode.
  *
  * @param msg gets the mode from the broadcast
  */
  bool modeCallback(jetyak_uav_utils::Mode::Request  &req,
                    jetyak_uav_utils::Mode::Response &res);


  /****************************
  * SUBSCRIPTION CALLBACKS
  *****************************/


  /** tagPoseCallback
  * Changes the current mode
  * changes control_priority
  *
  * @param msg gets the pose of the tag relative to the UAV
  */
  void tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /** uavGPSCallback
  * Listens for updates from the UAVs GPS
  *
  * @param msg gets the global position of the UAV
  */
  void uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  /** boatGPSCallback
  * Listens for updates from the boat's GPS
  *
  * @param msg gets the global position of the boat
  */
  void boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  /** uavAttitudeCallback
  * Listens for updates in the UAVs Attitude
  *
  * @param msg gets the global attitude of the UAV
  */
  void uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

  /** uavImuCallback
  * Listens for updates in the UAVs IMU
  *
  * @param msg gets the Imu reading of the UAV
  */
  void uavImuCallback(const sensor_msgs::Imu::ConstPtr& msg);


  /** boatIMUCallback
  * Listens for updates in the boat's IMU
  *
  * @param msg gets the IMU reading of the boat
  */
  void boatIMUCallback(const sensor_msgs::Imu::ConstPtr& msg);


  /******************************
  * BEHAVIOR METHODS
  ******************************/
  /** takeoffBehavior
   * Take off and change to follow mode if successful
  */
  void takeoffBehavior();

  /** followBehavior
   * Follow the boat using tags (later fused sensors)
  */
  void followBehavior();
public:
  /** Constructor
  * Start up the Controller Node
  * Create publishers, subscribers, services
  *
  * @param nh node handler
  */
  behaviors(ros::NodeHandle& nh);


  ~behaviors();

  /** doBehaviorAction
   * calls the behavior designated by the mode service
  */
  void doBehaviorAction();


  /*************************
  * Reconfigure callbacks
  *************************/

  /** landReconfigureCallback
  * Listens for changes to the configuration of the Landing behaviors
  *
  * @param config Provides the configuration parameters which we will save for the landing method
  */
  void landReconfigureCallback(jetyak_uav_utils::LandConstantsConfig &config, uint32_t level);

  /** followReconfigureCallback
  * Listens for changes to the configuration of the Following behaviors
  *
  * @param config Provides the configuration parameters which we will save for the following method
  */
  void followReconfigureCallback(jetyak_uav_utils::FollowConstantsConfig &config, uint32_t level);

};

#endif
