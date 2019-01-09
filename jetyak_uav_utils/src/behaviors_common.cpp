#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_common.cpp
 *
 * Implements:
 * 	createPID
 * 	resetPID
 * 	setPID
 * 	importParams
 * 	assignPublishers
 * 	assignServiceClients
 * 	assignServiceServers
 * 	assignSubscribers
 */

void Behaviors::createPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki, bsc_common::pose4d_t &kd)
{
	xpid_ = new bsc_common::PID(follow_.kp.x, follow_.ki.x, follow_.kd.x);
	ypid_ = new bsc_common::PID(follow_.kp.y, follow_.ki.y, follow_.kd.y);
	zpid_ = new bsc_common::PID(follow_.kp.z, follow_.ki.z, follow_.kd.z);
	wpid_ = new bsc_common::PID(follow_.kp.w, follow_.ki.w, follow_.kd.w);
}

void Behaviors::resetPID()
{
	xpid_->reset();
	ypid_->reset();
	zpid_->reset();
	wpid_->reset();
}

void Behaviors::setPID(bsc_common::pose4d_t &kp, bsc_common::pose4d_t &ki, bsc_common::pose4d_t &kd)
{
	xpid_->updateParams(kp.x, ki.x, kd.x);
	ypid_->updateParams(kp.y, ki.y, kd.y);
	zpid_->updateParams(kp.z, ki.z, kd.z);
	wpid_->updateParams(kp.w, ki.w, kd.w);
}

void Behaviors::downloadParams(std::string ns_param)
{
	std::string ns;
	// If it ends in / or is only a ~, it is good to use
	if (ns_param.length() == 0 or ns_param.back() == '/' or ns_param.compare("~") == 0)
		ns = ns_param;
	else	// if it needs a / seperator, add it
		ns = ns_param + "/";

	// land pid
	ros::param::param<double>(ns + "land_x_kp", land_.kp.x, 0);
	ros::param::param<double>(ns + "land_y_kp", land_.kp.y, 0);
	ros::param::param<double>(ns + "land_z_kp", land_.kp.z, 0);
	ros::param::param<double>(ns + "land_w_kp", land_.kp.w, 0);

	ros::param::param<double>(ns + "land_x_kd", land_.kd.x, 0);
	ros::param::param<double>(ns + "land_y_kd", land_.kd.y, 0);
	ros::param::param<double>(ns + "land_z_kd", land_.kd.z, 0);
	ros::param::param<double>(ns + "land_w_kd", land_.kd.w, 0);

	ros::param::param<double>(ns + "land_x_ki", land_.ki.x, 0);
	ros::param::param<double>(ns + "land_y_ki", land_.ki.y, 0);
	ros::param::param<double>(ns + "land_z_ki", land_.ki.z, 0);
	ros::param::param<double>(ns + "land_w_ki", land_.ki.w, 0);

	ros::param::param<double>(ns + "land_x", land_.land_pose.x, 0);
	ros::param::param<double>(ns + "land_y", land_.land_pose.y, 0);
	ros::param::param<double>(ns + "land_z", land_.land_pose.z, 0);
	ros::param::param<double>(ns + "land_w", land_.land_pose.w, 0);

	double radius;
	ros::param::param<double>(ns + "land_height", land_.height_goal, 0);
	ros::param::param<double>(ns + "land_height_threshold", land_.threshold, 0);
	ros::param::param<double>(ns + "land_radius", radius, 0);
	land_.radiusSqr = radius * radius;

	// follow
	ros::param::param<double>(ns + "follow_x_kp", follow_.kp.x, 0);
	ros::param::param<double>(ns + "follow_y_kp", follow_.kp.y, 0);
	ros::param::param<double>(ns + "follow_z_kp", follow_.kp.z, 0);
	ros::param::param<double>(ns + "follow_w_kp", follow_.kp.w, 0);

	ros::param::param<double>(ns + "follow_x_kd", follow_.kd.x, 0);
	ros::param::param<double>(ns + "follow_y_kd", follow_.kd.y, 0);
	ros::param::param<double>(ns + "follow_z_kd", follow_.kd.z, 0);
	ros::param::param<double>(ns + "follow_w_kd", follow_.kd.w, 0);

	ros::param::param<double>(ns + "follow_x_ki", follow_.ki.x, 0);
	ros::param::param<double>(ns + "follow_y_ki", follow_.ki.y, 0);
	ros::param::param<double>(ns + "follow_z_ki", follow_.ki.z, 0);
	ros::param::param<double>(ns + "follow_w_ki", follow_.ki.w, 0);

	ros::param::param<double>(ns + "follow_x", follow_.follow_pose.x, 0);
	ros::param::param<double>(ns + "follow_y", follow_.follow_pose.y, 0);
	ros::param::param<double>(ns + "follow_z", follow_.follow_pose.z, 0);
	ros::param::param<double>(ns + "follow_w", follow_.follow_pose.w, 0);

	// Return
	double settleRadius;
	ros::param::param<double>(ns + "return_gotoHeight", return_.gotoHeight, 5);
	ros::param::param<double>(ns + "return_finalHeight", return_.finalHeight, 3);
	ros::param::param<double>(ns + "return_downRadius", return_.downRadius, 1);
	ros::param::param<double>(ns + "return_settleRadius", settleRadius, .5);
	ros::param::param<double>(ns + "return_tagTime", return_.tagTime, 1);
	ros::param::param<double>(ns + "return_tagLossThresh", return_.tagLossThresh, 5);
	return_.settleRadiusSquared = settleRadius * settleRadius;

	// takeoff
	ros::param::param<double>(ns + "takeoff_height", takeoff_.height, 0);
	ros::param::param<double>(ns + "takeoff_threshold", takeoff_.threshold, 0);
}

void Behaviors::uploadParams(std::string ns_param)
{
	std::string ns;
	// If it ends in / or is only a ~, it is good to use
	if (ns_param.length() == 0 or ns_param.back() == '/' or ns_param.compare("~") == 0)
		ns = ns_param;
	else	// if it needs a / seperator, add it
		ns = ns_param + "/";

	// land pid
	ros::param::set(ns + "land_x_kp", land_.kp.x);
	ros::param::set(ns + "land_y_kp", land_.kp.y);
	ros::param::set(ns + "land_z_kp", land_.kp.z);
	ros::param::set(ns + "land_w_kp", land_.kp.w);

	ros::param::set(ns + "land_x_kd", land_.kd.x);
	ros::param::set(ns + "land_y_kd", land_.kd.y);
	ros::param::set(ns + "land_z_kd", land_.kd.z);
	ros::param::set(ns + "land_w_kd", land_.kd.w);

	ros::param::set(ns + "land_x_ki", land_.ki.x);
	ros::param::set(ns + "land_y_ki", land_.ki.y);
	ros::param::set(ns + "land_z_ki", land_.ki.z);
	ros::param::set(ns + "land_w_ki", land_.ki.w);

	ros::param::set(ns + "land_x", land_.land_pose.x);
	ros::param::set(ns + "land_y", land_.land_pose.y);
	ros::param::set(ns + "land_z", land_.land_pose.z);
	ros::param::set(ns + "land_w", land_.land_pose.w);

	// follow
	ros::param::set(ns + "follow_x_kp", follow_.kp.x);
	ros::param::set(ns + "follow_y_kp", follow_.kp.y);
	ros::param::set(ns + "follow_z_kp", follow_.kp.z);
	ros::param::set(ns + "follow_w_kp", follow_.kp.w);

	ros::param::set(ns + "follow_x_kd", follow_.kd.x);
	ros::param::set(ns + "follow_y_kd", follow_.kd.y);
	ros::param::set(ns + "follow_z_kd", follow_.kd.z);
	ros::param::set(ns + "follow_w_kd", follow_.kd.w);

	ros::param::set(ns + "follow_x_ki", follow_.ki.x);
	ros::param::set(ns + "follow_y_ki", follow_.ki.y);
	ros::param::set(ns + "follow_z_ki", follow_.ki.z);
	ros::param::set(ns + "follow_w_ki", follow_.ki.w);

	ros::param::set(ns + "follow_x", follow_.follow_pose.x);
	ros::param::set(ns + "follow_y", follow_.follow_pose.y);
	ros::param::set(ns + "follow_z", follow_.follow_pose.z);
	ros::param::set(ns + "follow_w", follow_.follow_pose.w);

	// Return
	double settleRadius;
	ros::param::set(ns + "return_gotoHeight", return_.gotoHeight);
	ros::param::set(ns + "return_finalHeight", return_.finalHeight);
	ros::param::set(ns + "return_downRadius", return_.downRadius);
	ros::param::set(ns + "return_settleRadius", settleRadius);
	ros::param::set(ns + "return_tagTime", return_.tagTime);
	ros::param::set(ns + "return_tagLossThresh", return_.tagLossThresh);
	return_.settleRadiusSquared = settleRadius * settleRadius;

	// takeoff
	ros::param::set(ns + "takeoff_height", takeoff_.height);
	ros::param::set(ns + "takeoff_threshold", takeoff_.threshold);
}

void Behaviors::assignPublishers()
{
	cmdPub_ = nh.advertise<sensor_msgs::Joy>("behavior_cmd", 1);
}

void Behaviors::assignServiceClients()
{
	propSrv_ = nh.serviceClient<jetyak_uav_utils::SetBoolean>("prop_enable");
	takeoffSrv_ = nh.serviceClient<std_srvs::Trigger>("takeoff");
	landSrv_ = nh.serviceClient<std_srvs::Trigger>("land");
}

void Behaviors::assignServiceServers()
{
	setModeService_ = nh.advertiseService("setMode", &Behaviors::setModeCallback, this);
	getModeService_ = nh.advertiseService("getMode", &Behaviors::getModeCallback, this);
	setBoatNSService_ = nh.advertiseService("setBoatNS", &Behaviors::setBoatNSCallback, this);
	setFollowPIDService_ = nh.advertiseService("setFollowPID", &Behaviors::setFollowPIDCallback, this);
	setLandPIDService_ = nh.advertiseService("setLandPID", &Behaviors::setLandPIDCallback, this);
	setFollowPosition_ = nh.advertiseService("setFollowPosition", &Behaviors::setFollowPositionCallback, this);
	setLandPosition_ = nh.advertiseService("setLandPosition", &Behaviors::setLandPositionCallback, this);
	setTakeoffParams_ = nh.advertiseService("setTakeoffParams", &Behaviors::setTakeoffParamsCallback, this);
	setReturnParams_ = nh.advertiseService("setReturnParams", &Behaviors::setReturnParamsCallback, this);
	setLandParams_ = nh.advertiseService("setLandParams", &Behaviors::setLandParamsCallback, this);
}

void Behaviors::assignSubscribers()
{
	tagPoseSub_ = nh.subscribe("/tag_pose", 1, &Behaviors::tagPoseCallback, this);
	uavGPSSub_ = nh.subscribe("/dji_sdk/gps_position", 1, &Behaviors::uavGPSCallback, this);
	boatGPSSub_ = nh.subscribe("boat_gps", 1, &Behaviors::boatGPSCallback, this);
	uavAttSub_ = nh.subscribe("/dji_sdk/attitude", 1, &Behaviors::uavAttitudeCallback, this);
	boatIMUSub_ = nh.subscribe("boat_imu", 1, &Behaviors::boatIMUCallback, this);
}
