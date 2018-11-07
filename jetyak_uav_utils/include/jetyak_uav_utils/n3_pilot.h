#ifndef N3_PILOT_H
#define N3_PILOT_H

#include <string>

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

// DJI SDK includes
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk/SDKControlAuthority.h"
#include "dji_sdk/QueryDroneVersion.h"

#define C_PI (double)3.141592653589793

class n3_pilot
{
public:
	n3_pilot(ros::NodeHandle& nh);
	~n3_pilot();

	void publishCommand();
protected:
	// ROS Subscribers
	ros::Subscriber joySub;
	ros::Subscriber djiRCSub;

	// ROS Publishers
	ros::Publisher controlPub;

	// ROS Services
	ros::ServiceClient sdkCtrlAuthorityServ;
	ros::ServiceClient droneVersionServ;

	// Callback functions
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void rcCallback(const sensor_msgs::Joy::ConstPtr& msg);

	// Functions
	void setupRCCallback();
	bool requestControl(int requestFlag);
	void setClipingThresholds();
	void adaptiveCliping();
	bool versionCheckM100();

	// Data
	sensor_msgs::Joy joyCommand, rcCommand;
	bool autopilotOn;
	bool bypassPilot;
	uint8_t commandFlag;

	int modeFlag, pilotFlag;
	double hVelcmdMaxBody, hVelcmdMaxGround, hARatecmdMax, hAnglecmdMax;
	double vVelcmdMaxBody, vVelcmdMaxGround, vPoscmdMax, vPoscmdMin, vThrustcmdMax;
	double yARateMax, yAngleMax;
	double rcStickThresh;

	bool isM100;
};

#endif