#include "jetyak_uav_utils/behaviors.h"
/** @file behaviors_behaviors.cpp
 *
 * Implements behaviors:
 * 	takeoffBehavior
 * 	followBehavior
 * 	returnBehavior
 * 	landBehavior
 * 	rideBehavior
 * 	hoverBehavior
 */

void Behaviors::takeoffBehavior()
{
	/*
	Get altitude/tagpose

	Start props
	PID using follow constants to 1 m above takeoff
	Enter following when altitude above .75m

	*/
	if (!propellorsRunning)
	{
		takeoff_.boatz = simpleTag_.z;
		std_srvs::Trigger srv;

		takeoffSrv_.call(srv);
		if (srv.response.success)
		{
			ROS_INFO("Propellors running");
			// behaviorChanged_=true;
			// currentMode_=Mode::FOLLOW;
			propellorsRunning = true;

			resetPID();
			setPID(follow_.kp, follow_.ki, follow_.kd);
		}
		else
		{
			ROS_WARN("Failure to Start props");
		}
	}
	else
	{
		if (takeoff_.boatz - simpleTag_.z > takeoff_.height)
		{
			ROS_WARN("Switching to Following");
			behaviorChanged_ = true;
			this->currentMode_ = JETYAK_UAV::FOLLOW;
		}
		else
		{
			xpid_->update(land_.goal_pose.x - simpleTag_.x, simpleTag_.t);
			ypid_->update(land_.goal_pose.y - simpleTag_.y, simpleTag_.t);
			zpid_->update((takeoff_.boatz - takeoff_.height) - simpleTag_.z, simpleTag_.t);
			wpid_->update(land_.goal_pose.w - simpleTag_.w, simpleTag_.t);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(-xpid_->get_signal());
			cmd.axes.push_back(-ypid_->get_signal());
			cmd.axes.push_back(-zpid_->get_signal());
			cmd.axes.push_back(-wpid_->get_signal());
			cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
			cmdPub_.publish(cmd);
		}
	}
}

void Behaviors::followBehavior()
{
	if (behaviorChanged_)
	{	// if just changed
		follow_.lastSpotted = simpleTag_.t;
		follow_.lostTagCounter = 0;
		resetPID();
		setPID(follow_.kp, follow_.ki, follow_.kd);
		behaviorChanged_ = false;
	}
	else
	{	// DO the loop

		if (follow_.lastSpotted != simpleTag_.t)
		{	// if time changed
			follow_.lastSpotted = simpleTag_.t;
			follow_.lostTagCounter = 0;

			// line up with pad
			xpid_->update(follow_.goal_pose.x - simpleTag_.x, simpleTag_.t);
			ypid_->update(follow_.goal_pose.y - simpleTag_.y, simpleTag_.t);
			zpid_->update(follow_.goal_pose.z - simpleTag_.z, simpleTag_.t);
			wpid_->update(follow_.goal_pose.w - simpleTag_.w, simpleTag_.t);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(-xpid_->get_signal());
			cmd.axes.push_back(-ypid_->get_signal());
			cmd.axes.push_back(-zpid_->get_signal());
			cmd.axes.push_back(-wpid_->get_signal());
			cmd.axes.push_back(JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::BODY_FRAME);
			cmdPub_.publish(cmd);
		}
		else
		{	// if time is same
			follow_.lostTagCounter++;

			if (follow_.lostTagCounter > 10)
			{	// if time has been same for over 3 tick

				ROS_WARN("Tag Lost");
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
				cmdPub_.publish(cmd);
				return;
			}
		}
	}
}

void Behaviors::leaveBehavior()
{
	cmdPub_.publish(leave_.input);
}

void Behaviors::returnBehavior()
{
	double east = bsc_common::util::latlondist(0, boatGPS_.longitude, 0, uavGPS_.longitude);
	double north = bsc_common::util::latlondist(boatGPS_.latitude, 0, uavGPS_.latitude, 0);

	if (boatGPS_.longitude < uavGPS_.longitude)
		east = -east;
	if (boatGPS_.latitude < uavGPS_.latitude)
		north = -north;

	double heading = atan2(north, east);
	ROS_WARN("Heading: %1.3f", heading);

	if (ros::Time::now().toSec() - tagPose_.header.stamp.toSec() < return_.tagTime)
	{
		ROS_WARN("Tag Spotted");
		return_.stage = return_.SETTLE;
		if ((pow(follow_.goal_pose.x - simpleTag_.x, 2) + pow(follow_.goal_pose.y - simpleTag_.y, 2) +
				 pow(follow_.goal_pose.z - simpleTag_.z, 2)) < return_.settleRadiusSquared)
		{
			ROS_WARN("Settled, now following");
			currentMode_ = JETYAK_UAV::FOLLOW;
			behaviorChanged_ = true;
		}
		else
		{	// line up with pad
			xpid_->update(follow_.goal_pose.x - simpleTag_.x, simpleTag_.t);
			ypid_->update(follow_.goal_pose.y - simpleTag_.y, simpleTag_.t);
			zpid_->update(return_.finalHeight - simpleTag_.z, simpleTag_.t);
			wpid_->update(follow_.goal_pose.w - simpleTag_.w, simpleTag_.t);

			sensor_msgs::Joy cmd;
			cmd.axes.push_back(-xpid_->get_signal());
			cmd.axes.push_back(-ypid_->get_signal());
			cmd.axes.push_back(-zpid_->get_signal());
			cmd.axes.push_back(-wpid_->get_signal());
			cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
			cmdPub_.publish(cmd);
		}
	}
	else if (return_.stage == return_.SETTLE and
					 ros::Time::now().toSec() - tagPose_.header.stamp.toSec() > return_.tagLossThresh)
	{
		return_.stage = return_.UP;
	}

	else if (behaviorChanged_)
	{
		/*
		set stage to up
		set behavior changed to false
		init pid
	*/
		ROS_WARN("Behavior is now return");
		resetPID();
		setPID(follow_.kp, follow_.ki, follow_.kd);
		behaviorChanged_ = false;
		return_.stage = return_.UP;
		ROS_WARN("Going Up");
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}

	else if (return_.stage == return_.UP)
	{
		if (uavHeight_ >= return_.gotoHeight)
		{
			ROS_WARN("Changed OVER");
			return_.stage = return_.OVER;
		}
		else
		{
			double z_correction = return_.gotoHeight - uavHeight_;
			ROS_WARN("Goal: %1.3f, Current %1.3f", return_.gotoHeight, uavHeight_);
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(0);
			cmd.axes.push_back(0);
			cmd.axes.push_back(z_correction);
			cmd.axes.push_back(0);
			cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD);
			cmdPub_.publish(cmd);
		}
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}
	else if (return_.stage == return_.OVER)
	{
		/*
		saturate x and y with directions
		z_cmd = gotoHeight-altitude
		if in sphere around goal
			set stage to DOWN
	*/
		if (bsc_common::util::latlondist(uavGPS_.latitude, uavGPS_.longitude, boatGPS_.latitude, boatGPS_.longitude) <
				return_.downRadius)
		{
			ROS_WARN("Changed DOWN");
			return_.stage = return_.DOWN;
		}
		else
		{
			double z_correction = follow_.kp.z * (return_.gotoHeight - uavHeight_);
			double mag = sqrt(east*east+north*north);
			if(mag > return_.maxVel) {
				east = east*return_.maxVel/mag;
				north = north*return_.maxVel/mag;
			}
			sensor_msgs::Joy cmd;
			cmd.axes.push_back(east);
			cmd.axes.push_back(north);
			cmd.axes.push_back(z_correction);
			cmd.axes.push_back(0);
			cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD);
			cmdPub_.publish(cmd);
		}
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}
	else if (return_.stage = return_.DOWN)
	{
		/*
		find proper pid constants to maintain location more or less
		z=finalHeight-altitude
		we wont change the mode here as it should search for the tags and
		hopefully find one
	*/
		double z_correction = follow_.kp.z * (return_.finalHeight - uavHeight_);

		ROS_WARN("Goal: %1.3f, Current %1.3f", return_.finalHeight, uavHeight_);

		sensor_msgs::Joy cmd;
		cmd.axes.push_back(east);
		cmd.axes.push_back(north);
		cmd.axes.push_back(z_correction);
		cmd.axes.push_back(0);
		cmd.axes.push_back(JETYAK_UAV::WORLD_FRAME | JETYAK_UAV::VELOCITY_CMD);
		cmdPub_.publish(cmd);
		std_srvs::Trigger downSrvTmp;
		lookdownSrv_.call(downSrvTmp);
	}
	else
	{
		ROS_ERROR("BAD CONDITIONALS, DEFAULTING TO HOVER");
		currentMode_ = JETYAK_UAV::HOVER;
	}
};

void Behaviors::landBehavior()
{
	if (behaviorChanged_)
	{	// if just changed
		land_.lastSpotted = simpleTag_.t;
		land_.lostTagCounter = 0;
		resetPID();
		setPID(land_.kp, land_.ki, land_.kd);
		behaviorChanged_ = false;
	}
	else
	{	// DO the loop

		if (follow_.lastSpotted != simpleTag_.t)
		{	// if time changed

			// If pose is within a cylinder of radius .1 and height .1
			bool inHeightThreshold = land_.goal_pose.z - simpleTag_.z < land_.heightThresh;
			bool inRadiusThreshhold =
					pow(land_.goal_pose.x - simpleTag_.x, 2) + pow(land_.goal_pose.y - simpleTag_.y, 2) < land_.radiusThreshSqr;
			bool inVelThreshold = pow(tagVel_.x, 2) + pow(tagVel_.y, 2) + pow(tagVel_.z, 2) < land_.velThreshSqr;
			bool inAngleThreshhold = abs(simpleTag_.w) < land_.angleThresh;
			if (inHeightThreshold and inRadiusThreshhold and inVelThreshold and inAngleThreshhold)
			{
				std_srvs::Trigger srv;
				landSrv_.call(srv);
				if (srv.response.success)
				{
					currentMode_ = JETYAK_UAV::RIDE;
					return;
				}
			}

			land_.lastSpotted = simpleTag_.t;
			land_.lostTagCounter = 0;
		}
		else
		{	// if time is same
			follow_.lostTagCounter++;

			if (follow_.lostTagCounter > 3 and follow_.lostTagCounter <= 20)
			{	// if time has been same for over 3 tick
				ROS_WARN("No tag update: %i", follow_.lostTagCounter);
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(.5);	// If tag lost, fly up a bit
				cmd.axes.push_back(0);
				cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
				cmdPub_.publish(cmd);
				return;
			}
			else if (follow_.lostTagCounter > 20)
			{	// if time has been same for over 10 tick
				ROS_WARN("Tag Lost");
				sensor_msgs::Joy cmd;
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(0);
				cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
				cmdPub_.publish(cmd);
				return;
			}
		}

		// line up with pad
		xpid_->update(land_.goal_pose.x - simpleTag_.x, simpleTag_.t);
		ypid_->update(land_.goal_pose.y - simpleTag_.y, simpleTag_.t);
		zpid_->update(land_.goal_pose.z - simpleTag_.z, simpleTag_.t);
		wpid_->update(land_.goal_pose.w - simpleTag_.w, simpleTag_.t);

		sensor_msgs::Joy cmd;
		cmd.axes.push_back(-xpid_->get_signal());
		cmd.axes.push_back(-ypid_->get_signal());
		cmd.axes.push_back(-zpid_->get_signal());
		cmd.axes.push_back(-wpid_->get_signal());
		cmd.axes.push_back(JETYAK_UAV::VELOCITY_CMD | JETYAK_UAV::BODY_FRAME);
		cmdPub_.publish(cmd);
	}
};

void Behaviors::rideBehavior()
{
	if (propellorsRunning)
	{
		std_srvs::Trigger srv;
		landSrv_.call(srv);
		propellorsRunning = srv.response.success;
		if (srv.response.success)
		{
			ROS_WARN("Arms deactivated");
		}
		else
		{
			ROS_WARN("Failed to deactivate arms");
		}
	}
	return_.altitudeOffset =
			uavGPS_.altitude - boatGPS_.altitude - return_.altitudeOffset;	// update offset TODO: add filter or rolling avg
}

void Behaviors::hoverBehavior()
{
	sensor_msgs::Joy cmd;
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(0);
	cmd.axes.push_back(JETYAK_UAV::BODY_FRAME | JETYAK_UAV::VELOCITY_CMD);
	cmdPub_.publish(cmd);
};
