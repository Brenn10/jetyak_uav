#include "jetyak_uav_utils/n3_pilot.h"

n3_pilot::n3_pilot(ros::NodeHandle& nh)
{
	// Subscribe to joy topic
	joySub = nh.subscribe("behavior_cmd", 10, &n3_pilot::joyCallback, this);
	djiRCSub = nh.subscribe("/dji_sdk/rc", 10, &n3_pilot::rcCallback, this);

	// Set up command publisher
	controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

	// Set up basic services
	sdkCtrlAuthorityServ = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
	droneVersionServ = nh.serviceClient<dji_sdk::QueryDroneVersion>("/dji_sdk/query_drone_version");

	// Set default values
	autopilotOn = false;
	bypassPilot = false;
	isM100 = true; //versionCheckM100();
	
	// Initialize RC
	setupRCCallback();

	// Initialize joy command
	joyCommand.axes.clear();
	for (int i = 0; i < 5; ++i)
		joyCommand.axes.push_back(0);

	// Initialize default command flag
	commandFlag = (
		DJISDK::VERTICAL_VELOCITY   |
		DJISDK::HORIZONTAL_VELOCITY |
		DJISDK::YAW_RATE            |
		DJISDK::HORIZONTAL_BODY     |
		DJISDK::STABLE_ENABLE);
}

n3_pilot::~n3_pilot()
{
	if (autopilotOn)
	{
		// Try to release control
		if(requestControl(0))
			ROS_INFO("Control released back to RC");
	}
}

// Callbacks //

void n3_pilot::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Pass the joystick message to the command
	joyCommand.axes.clear();
	joyCommand.axes.push_back(msg->axes[0]); // Roll
	joyCommand.axes.push_back(msg->axes[1]); // Pitch
	joyCommand.axes.push_back(msg->axes[2]); // Altitude
	joyCommand.axes.push_back(msg->axes[3]); // Yaw
	joyCommand.axes.push_back(msg->axes[4]); // Command Flag
}

void n3_pilot::rcCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Switch autopilot on/off
	// P mode && Autopilot switch on && Autopilot flag not set
	if (msg->axes[4] == modeFlag && msg->axes[5] == pilotFlag && !autopilotOn)
	{
		if(requestControl(1))
			autopilotOn = true;
	}
	// P mode && Autopilot switch off && Autopilot flag set
	// Not P mode && Autopilot flag set
	else if ((msg->axes[4] == modeFlag && msg->axes[5] != pilotFlag && autopilotOn) ||
		(msg->axes[4] != modeFlag && autopilotOn))
	{
		if(requestControl(0))
			autopilotOn = false;
	}

	// If it is on P mode and the autopilot is on check if the RC is being used
	if (msg->axes[4] == modeFlag && autopilotOn)
	{
		if(msg->axes[0] != 0 || msg->axes[1] != 0 || msg->axes[2] != 0 || msg->axes[3] != 0)
		{
			// Clear any previous RC commands
			rcCommand.axes.clear();
			rcCommand.axes.push_back(msg->axes[1]);  // Roll
			rcCommand.axes.push_back(-msg->axes[0]); // Pitch
			rcCommand.axes.push_back(msg->axes[3]);  // Altitude
			rcCommand.axes.push_back(-msg->axes[2]); // Yaw
			rcCommand.axes.push_back(commandFlag);   // Command Flag

			bypassPilot = true;
		}
	}
}

// Private //

void n3_pilot::setupRCCallback()
{
	if (isM100)
	{
		modeFlag = 8000;
		pilotFlag = -10000;
	}
	else
	{
		modeFlag = 1;
		pilotFlag = 1;
	}
}

bool n3_pilot::requestControl(int requestFlag)
{
	// Create control request and transmit it to vehicle
	dji_sdk::SDKControlAuthority ctrlAuthority;
	ctrlAuthority.request.control_enable = requestFlag;

	// Request control
	sdkCtrlAuthorityServ.call(ctrlAuthority);

	if(!ctrlAuthority.response.result)
	{
		ROS_ERROR("Could not switch control");
		return false;
	}
	else
	{
		if (requestFlag)
			ROS_INFO("Control of vehicle is obtained");
		else
			ROS_INFO("Released vehicle control");
	}

	return true;
}

bool n3_pilot::versionCheckM100()
{
	dji_sdk::QueryDroneVersion query;
	droneVersionServ.call(query);

	if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
		return true;

	return false;
}

void n3_pilot::publishCommand()
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

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "n3_pilot_node");
  ros::NodeHandle nh;

  n3_pilot joyPilot(nh);

  ros::Rate rate(10);

  while(ros::ok())
  {
  	ros::spinOnce();

  	joyPilot.publishCommand();

  	rate.sleep();
  }

  return 0;
}
