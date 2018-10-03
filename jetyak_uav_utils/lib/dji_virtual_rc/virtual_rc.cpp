#include "include/virtual_rc.h"

virtual_rc::virtual_rc(ros::NodeHandle& nh)
{
	// Subscribe to joy topic
	setJoyTopic(nh, "behavior_cmd");
	djiRCSub = nh.subscribe("/dji_sdk/rc", 10, &virtual_rc::rcCallback, this);

	// Set up command publisher
	controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

	// Set up basic services
	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_permission_control");

	// Set default values
	autopilotOn = false;
	bypassPilot = false;

	commandFlag = (
		DJISDK::VERTICAL_VELOCITY   |
		DJISDK::HORIZONTAL_VELOCITY |
		DJISDK::YAW_RATE            |
		DJISDK::HORIZONTAL_BODY     |
		DJISDK::STABLE_ENABLE);
}

virtual_rc::~virtual_rc()
{
	if (autopilotOn)
	{
		// Try to release control
		if(releaseControl())
			ROS_INFO("Control released back to RC");
	}
}

void virtual_rc::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Pass the joystick message to the command
	joyCommand.axes.clear();
	joyCommand.axes.push_back(msg->axes[0]); // Roll
	joyCommand.axes.push_back(msg->axes[1]); // Pitch
	joyCommand.axes.push_back(msg->axes[2]); // Altitude
	joyCommand.axes.push_back(msg->axes[3]); // Yaw
	joyCommand.axes.push_back(msg->axes[4]); // Command Flag
}

void virtual_rc::rcCallback(const sensor_msgs::Joy::ConstPtr& msg)
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

bool virtual_rc::requestControl()
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

bool virtual_rc::releaseControl()
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

void virtual_rc::setJoyTopic(ros::NodeHandle& nh, std::string topicJoy)
{
	// Terminate any existing subscriptions
	joySub.shutdown();

	joyTopic = topicJoy;
	joySub = nh.subscribe(joyTopic, 10, &virtual_rc::joyCallback, this);

}

void virtual_rc::publishCommand()
{
	if(autopilotOn)
	{
		if(bypassPilot)
			controlPub.publish(rcCommand);
		else
			controlPub.publish(joyCommand);

		// Reset bypass flag
		bypassPilot = false;
	}
}
