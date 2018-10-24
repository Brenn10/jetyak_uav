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
#include "Mode.h"
#include "jetyak_uav_utils/SetMode.h"
#include "jetyak_uav_utils/GetMode.h"

//ROS Core includes
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

//ROS Packages includes
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <dji_sdk/DroneArmControl.h>

//Custom Lib includes
#include "../lib/bsc_common/include/pid.h"
#include "../lib/bsc_common/include/util.h"
#include "../lib/bsc_common/include/pose4d.h"

class behaviors {
private:


  /*********************************************
  * ROS PUBLISHERS, SUBSCRIBERS, AND SERVICES
  *********************************************/
  ros::Subscriber tagPoseSub_, boatGPSSub_, boatIMUSub_, uavGPSSub_, uavAttSub_;
  ros::Publisher cmdPub_;
  ros::ServiceClient armSrv_;
  ros::ServiceServer setModeService_,getModeService_;

  /**********************
  * INSTANCE VARIABLES
  **********************/
  bsc_common::PID *xpid_,*ypid_,*zpid_,*wpid_; // pid controllers
  bool behaviorChanged_=false;
  Mode currentMode_;
  bool propellorsRunning=false;
  char bodyVelCmdFlag_ = (
    DJISDK::VERTICAL_VELOCITY   |
    DJISDK::HORIZONTAL_VELOCITY |
    DJISDK::YAW_RATE            |
    DJISDK::HORIZONTAL_BODY     |
    DJISDK::STABLE_DISABLE);

  char worldPositionCmdFlag_ = (
    DJISDK::VERTICAL_POSITION   |
    DJISDK::HORIZONTAL_POSITION |
    DJISDK::YAW_ANGLE            |
    DJISDK::HORIZONTAL_GROUND     |
    DJISDK::STABLE_ENABLE);

  /************************************
  * STATE VARIABLES
  ************************************/
  sensor_msgs::NavSatFix uavGPS_,boatGPS_;
  geometry_msgs::QuaternionStamped uavAttitude_;
  sensor_msgs::Imu uavImu_, boatImu_;
  geometry_msgs::PoseStamped tagPose_;

  bsc_common::pose4d_t actualPose_;

  /*********************************************
  * BEHAVIOR SPECIFIC VARIABLES AND CONSTANTS
  **********************************************/
  // Land specific constants
  struct {
    bsc_common::pose4d_t kp,kd,ki;
    double currGoalHeight;
    double collapseRatio;

  } land_;

  // follow specific constants
  struct {
    bsc_common::pose4d_t kp,kd,ki;
    bsc_common::pose4d_t follow_pose;
    double lastSpotted;
    int lostTagCounter;
  } follow_;

  /*********************
  * SERVICE CALLBACKS
  *********************/
  /** setModeCallback
  * Callback for the mode service. Changes the current mode.
  *
  * @param req mode to set
  * @param res successful
  */
  bool setModeCallback(jetyak_uav_utils::SetMode::Request  &req,
                    jetyak_uav_utils::SetMode::Response &res);
  /** getModeCallback
  * Callback for the mode service. Changes the current mode.
  *
  * @param req empty
  * @param res contains the mode
  */
  bool getModeCallback(jetyak_uav_utils::GetMode::Request  &req,
                    jetyak_uav_utils::GetMode::Response &res);

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

  /** returnBehavior
   * Safely return to the boat
  */
  void returnBehavior();

  /** landBehavior
   * Safely land on the boat
  */
  void landBehavior();

  /** rideBehavior
   * Safely ride on the boat
  */
  void rideBehavior();

  /** hoverBehavior
   * Safely hover
  */
  void hoverBehavior();
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




};

#endif
