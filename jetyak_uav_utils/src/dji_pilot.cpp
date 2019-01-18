#include "jetyak_uav_utils/dji_pilot.h"

#include <cmath>

#define C_PI (double)3.141592653589793
#define clip(X, LOW, HIGH) (((X) > (HIGH)) ? (HIGH) : ((X) < (LOW)) ? (LOW) : (X))

dji_pilot::dji_pilot(ros::NodeHandle &nh)
{
	// Subscribe to joy topic
	extCmdSub = nh.subscribe("behavior_cmd", 10, &dji_pilot::extCallback, this);

	djiRCSub = nh.subscribe("/dji_sdk/rc", 10, &dji_pilot::rcCallback, this);

	// Set up command publisher
	controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

	// Set up basic services
	// service clients
	armServ = nh.serviceClient<dji_sdk::DroneArmControl>("/dji_sdk/drone_arm_control");
	taskServ = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");

	sdkCtrlAuthorityServ = nh.serviceClient<dji_sdk::SDKControlAuthority>("/dji_sdk/sdk_control_authority");

	// set up service servers
	propServServer = nh.advertiseService("prop_enable", &dji_pilot::propServCallback, this);
	takeoffServServer = nh.advertiseService("takeoff", &dji_pilot::takeoffServCallback, this);
	landServServer = nh.advertiseService("land", &dji_pilot::landServCallback, this);

	// Set default values
	setClippingThresholds();
	rcStickThresh = 0.0;
	autopilotOn = false;
	bypassPilot = false;

	// Initialize RC
	if (!ros::param::get("~isM100", isM100))
	{
		isM100 = true;
		ROS_WARN("isM100 not available, defaulting to %i", isM100)
	}
}
setupRCCallback();

// Initialize joy command
extCommand.axes.clear();
for (int i = 0; i < 5; ++i)
	extCommand.axes.push_back(0);

// Initialize default command flag
commandFlag = (DJISDK::VERTICAL_VELOCITY | DJISDK::HORIZONTAL_VELOCITY | DJISDK::YAW_RATE | DJISDK::HORIZONTAL_BODY |
							 DJISDK::STABLE_ENABLE);
}

dji_pilot::~dji_pilot()
{
	if (autopilotOn)
	{
		// Try to release control
		if (requestControl(0))
			ROS_INFO("Control released back to RC");
	}
}

// Callbacks //
/** extCallback
 * @param msg Joy message containing rpty commands in the axes[0:3] and \
 * axes[4] encoding bools for body frame (0b10) and position cmd (0b01)
 */
void dji_pilot::extCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	// Check if incoming message is full
	if (msg->axes.size() == 5)
	{
		// Pass the joystick message to the command
		extCommand.axes.clear();
		sensor_msgs::Joy *output = new sensor_msgs::Joy();

		for (int i = 0; i < 5; i++)
			output->axes.push_back(msg->axes[i]);

		// Clip commands according to flag
		extCommand = adaptiveClipping(*output);
	}
}

void dji_pilot::rcCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	// Switch autodji_pilot on/off
	// P mode && Autodji_pilot switch on && Autodji_pilot flag not set
	if (msg->axes[4] == modeFlag && msg->axes[5] == pilotFlag && !autopilotOn)
	{
		if (requestControl(1))
			autopilotOn = true;
	}
	// P mode && Autodji_pilot switch off && Autodji_pilot flag set
	// Not P mode && Autodji_pilot flag set
	else if ((msg->axes[4] == modeFlag && msg->axes[5] != pilotFlag && autopilotOn) ||
					 (msg->axes[4] != modeFlag && autopilotOn))
	{
		if (requestControl(0))
			autopilotOn = false;
	}

	// If it is on P mode and the autodji_pilot is on check if the RC is being
	// used
	if (msg->axes[4] == modeFlag && autopilotOn)
	{
		if (std::abs(msg->axes[0]) > rcStickThresh || std::abs(msg->axes[1]) > rcStickThresh ||
				std::abs(msg->axes[2]) > rcStickThresh || std::abs(msg->axes[3]) > rcStickThresh)
		{
			// Clear any previous RC commands
			rcCommand.axes.clear();
			rcCommand.axes.push_back(msg->axes[1]);		// Roll
			rcCommand.axes.push_back(-msg->axes[0]);	// Pitch
			rcCommand.axes.push_back(msg->axes[3]);		// Altitude
			rcCommand.axes.push_back(-msg->axes[2]);	// Yaw
			rcCommand.axes.push_back(commandFlag);		// Command Flag

			bypassPilot = true;
		}
	}
}

bool dji_pilot::propServCallback(jetyak_uav_utils::SetBoolean::Request &req,
																 jetyak_uav_utils::SetBoolean::Response &res)
{
	if (autopilotOn)
	{
		dji_sdk::DroneArmControl srv;
		srv.request.arm = req.data;
		armServ.call(srv);
		res.success = srv.response.result;
		return true;
	}
	else
		return false;
}

bool dji_pilot::landServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	if (autopilotOn)
	{
		dji_sdk::DroneTaskControl srv;
		srv.request.task = 6;	// landing
		taskServ.call(srv);
		res.success = srv.response.result;
		return true;
	}
	else
		return false;
}

bool dji_pilot::takeoffServCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	if (autopilotOn)
	{
		dji_sdk::DroneTaskControl srv;
		srv.request.task = 4;	// takeoff
		taskServ.call(srv);
		res.success = srv.response.result;
		return true;
	}
	else
		return false;
}

// Private //

void dji_pilot::setupRCCallback()
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

bool dji_pilot::requestControl(int requestFlag)
{
	// Create control request and transmit it to vehicle
	dji_sdk::SDKControlAuthority ctrlAuthority;
	ctrlAuthority.request.control_enable = requestFlag;

	// Request control
	sdkCtrlAuthorityServ.call(ctrlAuthority);

	if (!ctrlAuthority.response.result)
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

void dji_pilot::setClippingThresholds()
{
	// Horizontal
	hVelcmdMaxBody = 1.0;							// m/sec
	hVelcmdMaxGround = 5.0;						// m/sec
	hARatecmdMax = 5.0 * C_PI / 6.0;	// rad/sec
	hAnglecmdMax = 0.611;							// rad

	// Vertical
	vVelcmdMaxBody = 1.0;		 // m/sec
	vVelcmdMaxGround = 3.0;	// m/sec
	vPoscmdMax = 30.0;			 // m
	vPoscmdMin = 0.0;				 // m
	vThrustcmdMax = 1.0;		 // *100% of max thrust

	// Yaw
	yARateMax = 5.0 * C_PI / 6.0;	// rad/sec
	yAngleMax = C_PI;							 // rad
}

sensor_msgs::Joy dji_pilot::adaptiveClipping(sensor_msgs::Joy msg)
{
	uint8_t flag = buildFlag((JETYAK_UAV::Flag)msg.axes[4]);

	// Create command buffer
	sensor_msgs::Joy cmdBuffer;
	cmdBuffer.axes.clear();

	for (int i = 0; i < 4; ++i)
		cmdBuffer.axes.push_back(0);
	cmdBuffer.axes.push_back(flag);

	// Coordinate frame
	double hVelcmdMax, vVelcmdMax;
	if ((flag & DJISDK::HORIZONTAL_BODY) == DJISDK::HORIZONTAL_BODY)
	{
		hVelcmdMax = hVelcmdMaxBody;
		vVelcmdMax = vVelcmdMaxBody;
	}
	else
	{
		hVelcmdMax = hVelcmdMaxGround;
		vVelcmdMax = vVelcmdMaxGround;
	}

	// Horizontal Logic
	if ((flag & DJISDK::HORIZONTAL_ANGULAR_RATE) == DJISDK::HORIZONTAL_ANGULAR_RATE)
	{
		cmdBuffer.axes[0] = clip(msg.axes[0], -hARatecmdMax, hARatecmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hARatecmdMax, hARatecmdMax);
	}
	else if ((flag & DJISDK::HORIZONTAL_POSITION) == DJISDK::HORIZONTAL_POSITION)
	{
		cmdBuffer.axes[0] = msg.axes[0];
		cmdBuffer.axes[1] = msg.axes[1];
	}
	else if ((flag & DJISDK::HORIZONTAL_VELOCITY) == DJISDK::HORIZONTAL_VELOCITY)
	{
		cmdBuffer.axes[0] = clip(msg.axes[0], -hVelcmdMax, hVelcmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hVelcmdMax, hVelcmdMax);
	}
	else
	{
		cmdBuffer.axes[0] = clip(msg.axes[0], -hAnglecmdMax, hAnglecmdMax);
		cmdBuffer.axes[1] = clip(msg.axes[1], -hAnglecmdMax, hAnglecmdMax);
	}

	// Vertical Logic
	if ((flag & DJISDK::VERTICAL_THRUST) == DJISDK::VERTICAL_THRUST)
		cmdBuffer.axes[3] = clip(msg.axes[3], 0, vThrustcmdMax);
	else if ((flag & DJISDK::VERTICAL_POSITION) == DJISDK::VERTICAL_POSITION)
		cmdBuffer.axes[3] = clip(msg.axes[3], vPoscmdMin, vPoscmdMax);
	else
		cmdBuffer.axes[3] = clip(msg.axes[3], -vVelcmdMax, vVelcmdMax);

	// Yaw Logic
	if ((flag & DJISDK::YAW_RATE) == DJISDK::YAW_RATE)
		cmdBuffer.axes[2] = clip(msg.axes[2], -yARateMax, yARateMax);
	else
		cmdBuffer.axes[2] = clip(msg.axes[2], -yAngleMax, yAngleMax);

	return cmdBuffer;
}

void dji_pilot::publishCommand()
{
	if (autopilotOn)
	{
		if (bypassPilot)
			controlPub.publish(rcCommand);
		else
			controlPub.publish(extCommand);

		// Reset bypass flag
		bypassPilot = false;
	}
}

uint8_t dji_pilot::buildFlag(JETYAK_UAV::Flag flag)
{
	uint8_t base = 0;

	if ((flag & JETYAK_UAV::POSITION_CMD) == JETYAK_UAV::POSITION_CMD)
		base |= DJISDK::HORIZONTAL_POSITION | DJISDK::VERTICAL_POSITION | DJISDK::YAW_ANGLE;
	else
		base |= DJISDK::HORIZONTAL_VELOCITY | DJISDK::VERTICAL_VELOCITY | DJISDK::YAW_RATE;

	if ((flag & JETYAK_UAV::BODY_FRAME) == JETYAK_UAV::BODY_FRAME)
		base |= DJISDK::HORIZONTAL_BODY | DJISDK::STABLE_DISABLE;
	else
		base |= DJISDK::HORIZONTAL_GROUND | DJISDK::STABLE_ENABLE;

	return base;
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dji_pilot_node");
	ros::NodeHandle nh;

	dji_pilot joydji_pilot(nh);

	ros::Rate rate(30);

	while (ros::ok())
	{
		ros::spinOnce();

		joydji_pilot.publishCommand();

		rate.sleep();
	}

	return 0;
}
