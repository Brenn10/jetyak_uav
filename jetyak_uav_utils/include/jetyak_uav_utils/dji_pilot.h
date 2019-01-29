#ifndef DJI_PILOT_H
#define DJI_PILOT_H

// System includes
#include <string>

// ROS
#include <ros/ros.h>

// ROS includes
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>

// DJI SDK includes
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/dji_sdk.h>

#include "jetyak_uav_utils/SetBoolean.h"
#include "jetyak_uav_utils/jetyak_uav.h"

class dji_pilot
{
public:
	dji_pilot(ros::NodeHandle &nh);
	~dji_pilot();

	void publishCommand();

protected:
	// ROS Subscribers
	ros::Subscriber extCmdSub;
	ros::Subscriber djiRCSub;

	// ROS Publishers
	ros::Publisher controlPub;

	// ROS Services
	ros::ServiceClient sdkCtrlAuthorityServ;
	ros::ServiceClient armServ, taskServ;
	ros::ServiceServer propServServer, takeoffServServer, landServServer;

	// Callback functions
	void extCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void rcCallback(const sensor_msgs::Joy::ConstPtr &msg);

	// Service Servers
	bool propServCallback(jetyak_uav_utils::SetBoolean::Request &req, jetyak_uav_utils::SetBoolean::Response &res);
	bool landServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool takeoffServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	
	// Functions
	void setupRCCallback();
	bool requestControl(int requestFlag);
	void loadPilotParameters();
	sensor_msgs::Joy adaptiveClipping(sensor_msgs::Joy msg);

	// Data
	sensor_msgs::Joy extCommand, rcCommand;
	bool autopilotOn;
	bool bypassPilot;
	uint8_t commandFlag;

	int modeFlag, pilotFlag;
	double hVelocityMaxBody, hVelocityMaxGround, hAngleRateCmdMax, hAngleCmdMax;
	double vVelocityMaxBody, vVelocityMaxGround, vPosCmdMax, vPosCmdMin, vThrustCmdMax;
	double yAngleRateMax, yAngleMax;
	double rcStickThresh;

	bool isM100;

private:
	uint8_t buildFlag(JETYAK_UAV::Flag flag);
};

#endif
