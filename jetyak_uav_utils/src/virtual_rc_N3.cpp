#include "jetyak_uav_utils/virtual_rc_N3.h"

void virtual_rc_N3::rcCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Switch autopilot on/off
	// F mode && Autopilot switch on && Autopilot flag not set
	if (msg->axes[4] == 1 && msg->axes[5] == 1 && !autopilotOn)
	{
		if(requestControl())
			autopilotOn = true;
	}
	// F mode && Autopilot switch off && Autopilot flag set
	// P mode || A mode && Autopilot flag set
	else if ((msg->axes[4] == 1 && msg->axes[5] != 1 && autopilotOn) ||
		(msg->axes[4] != 1 && autopilotOn))
	{
		if(releaseControl())
			autopilotOn = false;
	}

	// If it is on F mode and the autopilot is on check if the RC is being used
	if (msg->axes[4] == 1 && autopilotOn)
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

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "virtual_rc_N3");
  ros::NodeHandle nh;

  virtual_rc_N3 joyPilot(nh);
  joyPilot.setJoyTopic(nh, "joy");

  ros::Rate rate(10);

  while(ros::ok())
  {
  	ros::spinOnce();

  	joyPilot.publishCommand();

  	rate.sleep();
  }

  return 0;
}