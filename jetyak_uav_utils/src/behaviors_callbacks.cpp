#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_callbacks.cpp
 *
 * Implements subscriber callbacks:
 * 	tagPoseCallback
 * 	uavGPSCallback
 * 	boatGPSCallback
 * 	uavAttitudeCallback
 * 	uavImuCallback
 * 	boatIMUCallback
 */
void Behaviors::tagPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	tagPose_.pose = msg->pose;
	tagPose_.header = msg->header;
}

void Behaviors::uavGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	if (msg->status.status >= 0)
	{
		uavGPS_.header = msg->header;
		uavGPS_.status = msg->status;
		uavGPS_.latitude = msg->latitude;
		uavGPS_.longitude = msg->longitude;
		uavGPS_.altitude = msg->altitude;
		uavGPS_.position_covariance = msg->position_covariance;
		uavGPS_.position_covariance_type = msg->position_covariance_type;
	}
	else
	{
		ROS_WARN("UAV GNSS fix failed. Status: %i", msg->status.status);
	}
}

void Behaviors::boatGPSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	if (msg->status.status >= 0)
	{
		boatGPS_.header = msg->header;
		boatGPS_.status = msg->status;
		boatGPS_.latitude = msg->latitude;
		boatGPS_.longitude = msg->longitude;
		boatGPS_.altitude = msg->altitude;
		boatGPS_.position_covariance = msg->position_covariance;
		boatGPS_.position_covariance_type = msg->position_covariance_type;
	}
	else
	{
		ROS_WARN("Boat GNSS fix failed. Status: %i", msg->status.status);
	}
}

void Behaviors::uavAttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
	uavAttitude_.header = msg->header;
	uavAttitude_.quaternion = msg->quaternion;
}

void Behaviors::uavImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	uavImu_.header = msg->header;
	uavImu_.orientation = msg->orientation;
	uavImu_.orientation_covariance = msg->orientation_covariance;
	uavImu_.angular_velocity = msg->angular_velocity;
	uavImu_.angular_velocity_covariance = msg->angular_velocity_covariance;
	uavImu_.linear_acceleration = msg->linear_acceleration;
	uavImu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void Behaviors::boatIMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	boatImu_.header = msg->header;
	boatImu_.orientation = msg->orientation;
	boatImu_.orientation_covariance = msg->orientation_covariance;
	boatImu_.angular_velocity = msg->angular_velocity;
	boatImu_.angular_velocity_covariance = msg->angular_velocity_covariance;
	boatImu_.linear_acceleration = msg->linear_acceleration;
	boatImu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}
