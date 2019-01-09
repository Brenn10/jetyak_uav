/** Base behavioral controller for the uav
 * Manages the high level methods of the system.
 * Sends commands to the UAV
 * implements a joystick ovveride
 * implements a safety controller for if the drone is too close to an object
 * Listens to topic to determine when to switch modes
 */

#ifndef JETYAK_UAV_UTILS_BEHAVIORS_H_
#define JETYAK_UAV_UTILS_BEHAVIORS_H_

// C includes
#include <cmath>
#include <cstdlib>
#include <vector>

// ROS
#include <ros/ros.h>

// ROS Core includes
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Trigger.h>

// Jetyak UAV Includes
#include "jetyak_uav_utils/FourAxes.h"
#include "jetyak_uav_utils/GetString.h"
#include "jetyak_uav_utils/LandParams.h"
#include "jetyak_uav_utils/ReturnParams.h"
#include "jetyak_uav_utils/SetBoolean.h"
#include "jetyak_uav_utils/SetString.h"
#include "jetyak_uav_utils/TakeoffParams.h"
#include "jetyak_uav_utils/jetyak_uav.h"

// Lib includes
#include "../lib/bsc_common/include/pid.h"
#include "../lib/bsc_common/include/pose4d.h"
#include "../lib/bsc_common/include/util.h"

class Behaviors {
private:
  /*********************************************
   * ROS PUBLISHERS, SUBSCRIBERS, AND SERVICES
   *********************************************/
  ros::Subscriber tagPoseSub_, boatGPSSub_, boatIMUSub_, uavGPSSub_, uavAttSub_;
  ros::Publisher cmdPub_;
  ros::ServiceClient propSrv_, takeoffSrv_, landSrv_;
  ros::ServiceServer setModeService_, getModeService_, setBoatNSService_,
      setFollowPIDService_, setLandPIDService_, setFollowPosition_,
      setLandPosition_, setTakeoffParams_, setReturnParams_, setLandParams_;
  ros::NodeHandle nh;

  /**********************
   * INSTANCE VARIABLES
   **********************/
  bsc_common::PID *xpid_, *ypid_, *zpid_, *wpid_; // pid controllers
  bool behaviorChanged_ = false;
  JETYAK_UAV::Mode currentMode_;
  bool propellorsRunning = false;

  /************************************
   * STATE VARIABLES
   ************************************/
  sensor_msgs::NavSatFix uavGPS_, boatGPS_;
  geometry_msgs::QuaternionStamped uavAttitude_;
  sensor_msgs::Imu uavImu_, boatImu_;
  geometry_msgs::PoseStamped tagPose_ = geometry_msgs::PoseStamped();

  bsc_common::pose4d_t simpleTag_ = {0, 0, 0, 0, 0};

  /*********************************************
   * BEHAVIOR SPECIFIC VARIABLES AND CONSTANTS
   **********************************************/
  struct {
    double boatz;
    double height;
    double threshold;
  } takeoff_;

  // Land specific constants
  struct {
    bsc_common::pose4d_t kp, kd, ki;
    bsc_common::pose4d_t land_pose; // landing goal
    double lastSpotted;
    int lostTagCounter;
    double threshold;
    double radiusSqr;
  } land_;

  // follow specific constants
  struct {
    bsc_common::pose4d_t kp, kd, ki;
    bsc_common::pose4d_t follow_pose; // follow goal
    double lastSpotted;
    int lostTagCounter;
  } follow_;

  // follow specific constants
  struct {
    double gotoHeight;
    double finalHeight;
    double downRadius;
    double settleRadiusSquared = 1;
    double tagTime;
    double tagLossThresh;
    enum Stage { UP, OVER, DOWN, SETTLE } stage;
  } return_;

  /*********************
   * SERVICE CALLBACKS
   *********************/
  /** setModeCallback
   * Callback for the mode service. Changes the current mode.
   *
   * @param req mode to set
   * @param res successful
   */
  bool setModeCallback(jetyak_uav_utils::SetString::Request &req,
                       jetyak_uav_utils::SetString::Response &res);

  /** getModeCallback
   * Callback for the mode service. Changes the current mode.
   *
   * @param req empty
   * @param res contains the mode
   */
  bool getModeCallback(jetyak_uav_utils::GetString::Request &req,
                       jetyak_uav_utils::GetString::Response &res);

  /** setBoatNSCallback
   * Callback for the boat namespace service. This changes the topics for the
   * boat gps and imu subscriptions.
   *
   * @param req string of the root NS of the boat (ex. "/jetyak1")
   * @param res boolean indicating a success or failure
   */
  bool setBoatNSCallback(jetyak_uav_utils::SetString::Request &req,
                         jetyak_uav_utils::SetString::Response &res);

  /** setFollowPIDCallback
   * Change the constants in the follow pid controller
   *
   * @param req 4 arrays of new constants in P,I,D order for each axis
   * @param res boolean indicating a success or failure
   */
  bool setFollowPIDCallback(jetyak_uav_utils::FourAxes::Request &req,
                            jetyak_uav_utils::FourAxes::Response &res);

  /** setLandPIDCallback
   * Change the constants in the land pid controller
   *
   * @param req 4 arrays of new constants in P,I,D order for each axis
   * @param res boolean indicating a success or failure
   */
  bool setLandPIDCallback(jetyak_uav_utils::FourAxes::Request &req,
                          jetyak_uav_utils::FourAxes::Response &res);

  /** setFollowPositionCallback
   * Change the constants in the follow position
   *
   * @param req 4 arrays of a single value for position
   * @param res boolean indicating a success or failure
   */
  bool setFollowPositionCallback(jetyak_uav_utils::FourAxes::Request &req,
                                 jetyak_uav_utils::FourAxes::Response &res);

  /** setFollowPositionCallback
   * Change the constants in the land position
   *
   * @param req 4 arrays of a single value for position
   * @param res boolean indicating a success or failure
   */
  bool setLandPositionCallback(jetyak_uav_utils::FourAxes::Request &req,
                               jetyak_uav_utils::FourAxes::Response &res);

  /** setTakeoffParamsCallback
   * Change the constants for takeoff mode
   *
   * @param req param file
   * @param res boolean indicating a success or failure
   */
  bool setTakeoffParamsCallback(jetyak_uav_utils::TakeoffParams::Request &req,
                                jetyak_uav_utils::TakeoffParams::Response &res);

  /** setLandParamsCallback
   * Change the constants for land mode
   *
   * @param req param file
   * @param res boolean indicating a success or failure
   */
  bool setLandParamsCallback(jetyak_uav_utils::LandParams::Request &req,
                             jetyak_uav_utils::LandParams::Response &res);

  /** setReturnParamsCallback
   * Change the constants for return mode
   *
   * @param req param file
   * @param res boolean indicating a success or failure
   */
  bool setReturnParamsCallback(jetyak_uav_utils::ReturnParams::Request &req,
                               jetyak_uav_utils::ReturnParams::Response &res);

  /****************************
   * SUBSCRIPTION CALLBACKS
   *****************************/

  /** tagPoseCallback
   * Changes the current mode
   * changes control_priority
   *
   * @param msg gets the pose of the tag relative to the UAV
   */
  void tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /** uavGPSCallback
   * Listens for updates from the UAVs GPS
   *
   * @param msg gets the global position of the UAV
   */
  void uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

  /** boatGPSCallback
   * Listens for updates from the boat's GPS
   *
   * @param msg gets the global position of the boat
   */
  void boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

  /** uavAttitudeCallback
   * Listens for updates in the UAVs Attitude
   *
   * @param msg gets the global attitude of the UAV
   */
  void
  uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

  /** uavImuCallback
   * Listens for updates in the UAVs IMU
   *
   * @param msg gets the Imu reading of the UAV
   */
  void uavImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  /** boatIMUCallback
   * Listens for updates in the boat's IMU
   *
   * @param msg gets the IMU reading of the boat
   */
  void boatIMUCallback(const sensor_msgs::Imu::ConstPtr &msg);

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

  /*****************
   * Common Methods
   *****************/
  /** resetPID
   * Reset all PID controllers
   */
  void resetPID();

  /** setPID
   * Set constants for the controllers
   *
   * @param kp bsc_common::pose4d_t containing P constants
   * @param ki bsc_common::pose4d_t containing I constants
   * @param kd bsc_common::pose4d_t containing D constants
   */
  void setPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki,
              bsc_common::pose4d_t &kd);

  /** downloadParams
   * Download params from the namespaces's ros param server
   *
   * @param ns name of the namespace
   */
  void downloadParams(std::string ns = "");

  /** uploadParams
   * Upload params to the namespaces's ros param server
   *
   * @param ns name of the namespace
   */
  void uploadParams(std::string ns = "");

  /***********************
   * Constructor Methods
   **********************/
  /** createPID
   * Initialize and set constants for the controllers
   *
   * @param kp bsc_common::pose4d_t containing P constants
   * @param ki bsc_common::pose4d_t containing I constants
   * @param kd bsc_common::pose4d_t containing D constants
   */
  void createPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki,
                 bsc_common::pose4d_t &kd);

  void assignPublishers();
  void assignServiceClients();
  void assignServiceServers();
  void assignSubscribers();

public:
  /** Constructor
   * Start up the Controller Node
   * Create publishers, subscribers, services
   *
   * @param nh node handler
   */
  Behaviors(ros::NodeHandle &nh);

  ~Behaviors();

  /** doBehaviorAction
   * calls the behavior designated by the mode service
   */
  void doBehaviorAction();
};

#endif
