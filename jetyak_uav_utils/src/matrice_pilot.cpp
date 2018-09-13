#include "jetyak_uav_utils/matrice_pilot.h"
#include "dji_sdk/dji_sdk.h"

#include <iostream>

matrice_pilot::matrice_pilot(ros::NodeHandle& nh)
{
	// Subscribe to joy topic
	joySub = nh.subscribe("joy", 10, &matrice_pilot::joyCallback, this);
	djiRCSub = nh.subscribe("dji_sdk/rc", 10, &matrice_pilot::rcCallback, this);
	behaviorSub = nh.subscribe("jetyak_uav_utils/behaviorCmd", 10, &matrice_pilot::behaviorCallback, this);

	// Set up command publisher
	controlPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

	// Set up basic services
	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
	//query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion> ("dji_sdk/query_drone_version");

	// Init command flag
	commandFlag = (
		DJISDK::VERTICAL_VELOCITY   |
		DJISDK::HORIZONTAL_VELOCITY |
		DJISDK::YAW_RATE            |
		DJISDK::HORIZONTAL_BODY     |
		DJISDK::STABLE_ENABLE);

	joyDeadswitch = false;

	// Init command
	joyCommand.axes.clear();
	joyCommand.axes.push_back(0);
	joyCommand.axes.push_back(0);
	joyCommand.axes.push_back(0);
	joyCommand.axes.push_back(0);
	joyCommand.axes.push_back(commandFlag);

	// TO DO
	// Check that the control is released
	autopilotOn = false;
	bypassPilot = false;

	// TO DO
	// Add vehicle check
}

matrice_pilot::~matrice_pilot()
{
	if (autopilotOn)
	{
		// Try to release control
		if(releaseControl())
			ROS_INFO("Control released back to RC");
	}
}

////////////////////////////////////////////////////////////
///////////////  Subscription Callbacks  ///////////////////
////////////////////////////////////////////////////////////

void matrice_pilot::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	joyDeadswitch=(msg->buttons[4] | msg->buttons[4]);

	// Pass the joystick message to the command
	joyCommand.axes.clear();
	joyCommand.axes.push_back(msg->axes[4]); // Roll
	joyCommand.axes.push_back(msg->axes[3]); // Pitch
	joyCommand.axes.push_back(msg->axes[1]); // Altitude
	joyCommand.axes.push_back(msg->axes[0]); // Yaw
	joyCommand.axes.push_back(commandFlag);  // Command Flag
}

void matrice_pilot::rcCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// TO DO
	// This is specific to the vehicle controller. Add vehicle check and specify each flag

	// Switch autopilot on/off
	// F mode && Autopilot switch on && Autopilot flag not set
	if (msg->axes[4] == 8000 && msg->axes[5] == -10000 && !autopilotOn)
	{
		if(requestControl())
			autopilotOn = true;
	}
	// F mode && Autopilot switch off && Autopilot flag set
	// P mode || A mode && Autopilot flag set
	else if ((msg->axes[4] == 8000 && msg->axes[5] != -10000 && autopilotOn) ||
		(msg->axes[4] != 8000 && autopilotOn))
	{
		if(releaseControl())
			autopilotOn = false;
	}

	// If it is on F mode and the autopilot is on check if the RC is being used
	if (msg->axes[4] == 8000 && autopilotOn)
	{
		if(msg->axes[0] != 0 || msg->axes[1] != 0 || msg->axes[2] != 0 || msg->axes[3] != 0)
		{
			// Clear any previous RC commands
			rcCommand.axes.clear();
			rcCommand.axes.push_back(msg->axes[1]); // Roll
			rcCommand.axes.push_back(msg->axes[0]); // Pitch
			rcCommand.axes.push_back(msg->axes[3]); // Altitude
			rcCommand.axes.push_back(msg->axes[2]); // Yaw
			rcCommand.axes.push_back(commandFlag);  // Command Flag

			bypassPilot = true;
		}
	}
}

void matrice_pilot::behaviorCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Pass the joystick message to the command
	behaviorCommand.axes.clear();
	behaviorCommand.axes.push_back(msg->axes[0]); // Roll
	behaviorCommand.axes.push_back(msg->axes[1]); // Pitch
	behaviorCommand.axes.push_back(msg->axes[2]); // Altitude
	behaviorCommand.axes.push_back(msg->axes[3]); // Yaw
	behaviorCommand.axes.push_back(commandFlag);  // Command Flag
}

////////////////////////////////////////////////////////////
/////////////////  Private Functions  //////////////////////
////////////////////////////////////////////////////////////

bool matrice_pilot::requestControl()
{
	// Create control request and transmit it to vehicle
	dji_sdk::SDKControlAuthority ctrlAuthority;
	ctrlAuthority.request.control_enable = 1;

	// Request control
	sdk_ctrl_authority_service.call(ctrlAuthority);

	if(!ctrlAuthority.response.result)
	{
		ROS_ERROR("Could not obtain control");
		return false;
	}
	else
		ROS_INFO("Control of vehicle is obtained");

	return true;
}

bool matrice_pilot::releaseControl()
{
	// Create control request and transmit it to vehicle
	dji_sdk::SDKControlAuthority ctrlAuthority;
	ctrlAuthority.request.control_enable = 0;

	// Release control
	sdk_ctrl_authority_service.call(ctrlAuthority);

	if(!ctrlAuthority.response.result)
	{
		ROS_ERROR("Could not release control");
		return false;
	}

	return true;
}

void matrice_pilot::publishCommand()
{
	if(autopilotOn)
	{
		if(bypassPilot)
		{
			controlPub.publish(rcCommand);
		}
		else
		{
			if(joyDeadswitch)
				controlPub.publish(joyCommand);
			else
				controlPub.publish(behaviorCommand);
		}

		// Reset bypass flag
		bypassPilot = false;
	}
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrice_pilot");
  ros::NodeHandle nh;

  matrice_pilot joyPilot(nh);

  ros::Rate rate(10);

  while(ros::ok())
  {
  	ros::spinOnce();

  	joyPilot.publishCommand();

  	rate.sleep();
  }

  return 0;
}
