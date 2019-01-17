#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_main.cpp
 *
 * Implements structors and main functions:
 * 	Behaviors
 * 	~Behaviors
 * 	doBehaviorAction
 * 	main
 */
Behaviors::Behaviors(ros::NodeHandle &nh_param)
{
	nh = nh_param;

	// initialize mode
	currentMode_ = JETYAK_UAV::HOVER;

	assignSubscribers();
	assignPublishers();
	assignServiceClients();
	assignServiceServers();
	downloadParams();

	// initialize the tag
	tagP_.pose.orientation.x = 0;
	tagPose_.pose.orientation.y = 0;
	tagPose_.pose.orientation.z = 0;
	tagPose_.pose.orientation.w = 1;

	// Initialize the PID controllers
	createPID(follow_.kp, follow_.ki, follow_.kd);
}

Behaviors::~Behaviors()
{
}

void Behaviors::doBehaviorAction()
{
	simpleTag_.x = tagPose_.pose.position.x;
	simpleTag_.y = tagPose_.pose.position.y;
	simpleTag_.z = tagPose_.pose.position.z;

	simpleTag_.w = bsc_common::util::yaw_from_quat(tagPose_.pose.orientation);

	simpleTag_.t = tagPose_.header.stamp.toSec();

	/*ROS_INFO("x: %1.2f, y:%1.2f, z: %1.2f, yaw: %1.3f",
			simpleTag_.x,
			simpleTag_.y,
			simpleTag_.z,
			simpleTag_.w);*/

	// //Lets grab the most recent time stamp
	// if(uavGPS_.header.stamp.toSec()>boatGPS_.header.stamp.toSec())
	//   simpleTag_.header.stamp = uavGPS_.header.stamp;
	// else
	//   simpleTag_.header.stamp = uavGPS_.header.stamp;

	switch (currentMode_)
	{
		case JETYAK_UAV::TAKEOFF:
		{
			takeoffBehavior();
			break;
		}
		case JETYAK_UAV::FOLLOW:
		{
			followBehavior();
			break;
		}
		case JETYAK_UAV::LEAVE:
		{
			// Do nothing, an external node is currently communicating with the pilot
			break;
		}
		case JETYAK_UAV::RETURN:
		{
			returnBehavior();
			break;
		}
		case JETYAK_UAV::LAND:
		{
			landBehavior();
			break;
		}
		case JETYAK_UAV::RIDE:
		{
			rideBehavior();
			break;
		}
		case JETYAK_UAV::HOVER:
		{
			hoverBehavior();
			break;
		}
		default:
		{
			if (propellorsRunning)
			{
				ROS_ERROR("Mode out of bounds: %i. Now hovering.", (char)currentMode_);
				this->currentMode_ = JETYAK_UAV::HOVER;
			}
			else
			{
				ROS_ERROR("Mode out of bounds: %i. Now riding.", (char)currentMode_);
				this->currentMode_ = JETYAK_UAV::RIDE;
			}
			break;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "behaviors");
	ros::NodeHandle nh;
	Behaviors behaviors_o(nh);
	ros::Rate rate(30);

	while (ros::ok())
	{
		ros::spinOnce();

		behaviors_o.doBehaviorAction();

		rate.sleep();
	}
	return 0;
}
