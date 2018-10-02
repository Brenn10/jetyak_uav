//! The basic implementation of a virtual RC
/*!
	It subscribes to a "sensor_msgs::Joy" topic and passes the command to the
	"dji_sdk/flight_control_setpoint_generic" topic of the DJI SDK. The command
	is expected to have the correct structure
*/

#ifndef VIRTUAL_RC_H
#define VIRTUAL_RC_H

#include <string>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

// DJI SDK includes
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/SDKControlAuthority.h>

class virtual_rc
{
public:
	virtual_rc(){};
	~virtual_rc();

	//! Specify here the topic that the input commands will be published
	/*!
		\param nh       ros::NodeHandle
		\param topicJoy std::string the topic that input commands are published
	*/
	void setJoyTopic(ros::NodeHandle& nh, std::string topicJoy);

	void publishCommand();
protected:
	// ROS Subscribers
	ros::Subscriber joySub;
	ros::Subscriber djiRCSub;

	// ROS Publishers
	ros::Publisher controlPub;

	// ROS Services
	ros::ServiceClient sdk_ctrl_authority_service;

	// Callback functions
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void rcCallback(const sensor_msgs::Joy::ConstPtr& msg);

	// Functions
	void initializeRC(ros::NodeHandle &nh);
	bool requestControl();
	bool releaseControl();

	// Data
	std::string joyTopic;
	sensor_msgs::Joy joyCommand, rcCommand;
	bool autopilotOn;
	bool bypassPilot;
	uint8_t commandFlag;
};


#endif