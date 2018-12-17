#include "jetyak_uav_utils/n3_pilot.h"

#include <cmath>

#define C_PI (double)3.141592653589793
#define clip(X, LOW, HIGH) (((X) > (HIGH)) ? (HIGH) : ((X) < (LOW)) ? (LOW) : (X))

n3_pilot::n3_pilot(ros::NodeHandle& nh)
{
	// Subscribe to joy topic
	joySub = nh.subscribe("behavior_cmd", 10, &n3_pilot::joyCallback, this);

	djiRCSub = nh.subscribe("/dji_sdk/rc", 10, &n3_pilot::rcCallback, this);

	// Set up command publisher
	controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

	// Set up basic services
	//service clients
  armServ = nh.serviceClient<dji_sdk::DroneArmControl>("/dji_sdk/drone_arm_control");
  taskServ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

	sdkCtrlAuthorityServ = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
	droneVersionServ = nh.serviceClient<dji_sdk::QueryDroneVersion>("/dji_sdk/query_drone_version");

	//set up service servers
	armServServer = nh.advertiseService("arm_control",&n3_pilot::armServCallback,this);
	taskServServer = nh.advertiseService("task_control",&n3_pilot::taskServCallback,this);

	// Set default values
	setClippingThresholds();
	rcStickThresh = 0.0;
	autopilotOn = false;
	bypassPilot = false;
	//isM100 = false; //rsionCheckM100();
	ros::param::param<bool>("isM100", isM100, false);


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

	// Clip commands according to flag
	joyCommand = adaptiveClipping(*msg);
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
		if(std::abs(msg->axes[0]) > rcStickThresh || std::abs(msg->axes[1]) > rcStickThresh ||
		   std::abs(msg->axes[2]) > rcStickThresh || std::abs(msg->axes[3]) > rcStickThresh)
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

bool n3_pilot::armServCallback(dji_sdk::DroneArmControl::Request &req,
										 dji_sdk::DroneArmControl::Response &res)
{
	if(autopilotOn)
	{
		dji_sdk::DroneArmControl srv;
    srv.request.arm=req.arm;
    armServ.call(srv);
		res.result=srv.response.result;
		return true;
	}
	else
	{
		return false;
	}

}
bool n3_pilot::taskServCallback(dji_sdk::DroneTaskControl::Request &req,
										 dji_sdk::DroneTaskControl::Response &res)
{
	if(autopilotOn)
	{
		dji_sdk::DroneTaskControl srv;
    srv.request.task=req.task;
    taskServ.call(srv);
		res.result=srv.response.result;
		return true;
	}
	else
	{
		return false;
	}
}

// Private //

void n3_pilot::setupRCCallback()
{
	if (isM100) {
		modeFlag = 8000;
		pilotFlag = -10000;
	}
	else {
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

	if(!ctrlAuthority.response.result) {
		ROS_ERROR("Could not switch control");
		return false;
	}
	else {
		if (requestFlag)
			ROS_INFO("Control of vehicle is obtained");
		else
			ROS_INFO("Released vehicle control");
	}

	return true;
}

void n3_pilot::setClippingThresholds()
{
	// Horizontal
	hVelcmdMaxBody    = 1.0;           // m/sec
	hVelcmdMaxGround  = 5.0;           // m/sec
	hARatecmdMax      = 5.0*C_PI/6.0;  // rad/sec
	hAnglecmdMax      = 0.611;         // rad

	// Vertical
	vVelcmdMaxBody    = 1.0;           // m/sec
	vVelcmdMaxGround  = 3.0;           // m/sec
	vPoscmdMax        = 30.0;          // m
	vPoscmdMin        = 0.0;           // m
	vThrustcmdMax     = 1.0;           // *100% of max thrust

	// Yaw
	yARateMax         = 5.0*C_PI/6.0;  // rad/sec
	yAngleMax         = C_PI;          // rad
}

sensor_msgs::Joy n3_pilot::adaptiveClipping(sensor_msgs::Joy msg)
{
	// Create command buffer
	sensor_msgs::Joy cmdBuffer;
	cmdBuffer.axes.clear();
	for (int i = 0; i < 4; ++i)
		cmdBuffer.axes.push_back(0);
	cmdBuffer.axes.push_back(msg.axes[4]);

	// Coordinate frame
	double hVelcmdMax, vVelcmdMax;
	if (((int)msg.axes[4] & DJISDK::HORIZONTAL_BODY) ==
		DJISDK::HORIZONTAL_BODY) {
		hVelcmdMax = hVelcmdMaxBody;
		vVelcmdMax = vVelcmdMaxBody;
	}
	else {
		hVelcmdMax = hVelcmdMaxGround;
		vVelcmdMax = vVelcmdMaxGround;
	}

	// Horizontal Logic
	if(((int)msg.axes[4] & DJISDK::HORIZONTAL_ANGULAR_RATE) ==
		DJISDK::HORIZONTAL_ANGULAR_RATE) {
		cmdBuffer.axes[0] = clip(msg.axes[0], -hARatecmdMax, hARatecmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hARatecmdMax, hARatecmdMax);
	}
	else if(((int)msg.axes[4] & DJISDK::HORIZONTAL_POSITION) ==
		DJISDK::HORIZONTAL_POSITION) {
		cmdBuffer.axes[0] = msg.axes[0];
		cmdBuffer.axes[1] = msg.axes[1];
	}
	else if(((int)msg.axes[4] & DJISDK::HORIZONTAL_VELOCITY) ==
		DJISDK::HORIZONTAL_VELOCITY) {
		cmdBuffer.axes[0] = clip(msg.axes[0], -hVelcmdMax, hVelcmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hVelcmdMax, hVelcmdMax);
	}
	else {
		cmdBuffer.axes[0] = clip(msg.axes[0], -hAnglecmdMax, hAnglecmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hAnglecmdMax, hAnglecmdMax);
	}

	// Vertical Logic
	if(((int)msg.axes[4] & DJISDK::VERTICAL_THRUST) ==
		DJISDK::VERTICAL_THRUST)
		cmdBuffer.axes[3] = clip(msg.axes[3], 0, vThrustcmdMax);
	else if (((int)msg.axes[4] & DJISDK::VERTICAL_POSITION) ==
		DJISDK::VERTICAL_POSITION)
		cmdBuffer.axes[3] = clip(msg.axes[3], vPoscmdMin, vPoscmdMax);
	else
		cmdBuffer.axes[3] = clip(msg.axes[3], -vVelcmdMax, vVelcmdMax);

	// Yaw Logic
	if(((int)msg.axes[4] & DJISDK::YAW_RATE) == DJISDK::YAW_RATE)
		cmdBuffer.axes[2] = clip(msg.axes[2], -yARateMax, yARateMax);
	else
		cmdBuffer.axes[2] = clip(msg.axes[2], -yAngleMax, yAngleMax);

	return cmdBuffer;
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
