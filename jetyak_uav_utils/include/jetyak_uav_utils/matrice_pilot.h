/* Test controller for the DJI Matrice
*	Use a gamepad to control the Matrice
*/

#ifndef MATRICE_PILOT_H
#define MATRICE_PILOT_H

// STD includes
#include <iostream>

// ROS includes
#include "ros/ros.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/SDKControlAuthority.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

class matrice_pilot {
public:
	matrice_pilot(ros::NodeHandle& nh);
	~matrice_pilot();

	// Publisher
	void publishCommand();

private:
	// ROS Subscribers
	ros::Subscriber joySub;
	ros::Subscriber djiRCSub;
	ros::Subscriber behaviorSub;

	// ROS Publishers
	ros::Publisher controlPub;

	// ROS Services
	ros::ServiceClient sdk_ctrl_authority_service;
	//ros::ServiceClient query_version_service;

	// Callback functions
	 void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void rcCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void behaviorCallback(const sensor_msgs::Joy::ConstPtr& msg);

	// Functions
	bool requestControl();
	bool releaseControl();
	// bool is_M100();

	// Data
	sensor_msgs::Joy joyCommand, rcCommand,behaviorCommand;
	bool autopilotOn;
	bool joyDeadswitch;
	bool bypassPilot;
	uint8_t commandFlag;
};

#endif
