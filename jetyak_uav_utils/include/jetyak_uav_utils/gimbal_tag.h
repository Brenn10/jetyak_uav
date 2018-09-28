#ifndef GIMBAL_TAG_H
#define GIMBAL_TAG_H

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "tf/tf.h"

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

class gimbal_tag
{
public:
	gimbal_tag(ros::NodeHandle& nh);
	~gimbal_tag(){};
	
	// Publisher
	void publishTagPose();

private:
	// Subscribers
	ros::Subscriber tagPoseSub;
	ros::Subscriber gimbalAngleSub;
	ros::Subscriber vehicleAttiSub;

	// Publishers
	ros::Publisher tagBodyPosePub;

	// Callbacks
	void tagCallback(const ar_track_alvar_msgs::AlvarMarkers& msg);
	void gimbalCallback(const geometry_msgs::Vector3Stamped& msg);
	void attitudeCallback(const geometry_msgs::QuaternionStamped& msg);

	// Functions
	void updateTagPose();

	// Data
	tf::Quaternion qCamera2Gimbal;
	tf::Quaternion qConstant;
	tf::Quaternion qOffset;
	tf::Quaternion qGimbal;
	tf::Quaternion qVehicle;
	tf::Quaternion qTag;
	tf::Quaternion posTag;

	bool tagFound;
};

#endif